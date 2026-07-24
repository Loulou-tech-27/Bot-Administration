--[[
================================================================================
 FighterEscort.lua — Escorte de chasse à 2 avions IA pour X-Plane 12
================================================================================
 Framework : FlyWithLua NG+ (macOS / Windows / Linux)
 Emplacement : X-Plane 12/Resources/plugins/FlyWithLua/Scripts/FighterEscort.lua

 PRINCIPE / ARCHITECTURE
 -----------------------
 Deux approches étaient possibles :
   a) "non-invasive" : injecter des consignes dans l'autopilote/FMS des IA
      (impossible à fiabiliser : l'ATC interne reprend la main, refuse le
      décollage sans clairance, et ne sait pas tenir une formation serrée) ;
   b) "override natif" : écrire 1 dans le tableau
      sim/operation/override/override_planepath[n] pour l'avion IA n, ce qui
      DÉSACTIVE totalement le pilotage ATC/FMS interne de cet avion et laisse
      le script écrire directement sa position et son attitude à chaque frame.
 On retient (b) — c'est l'architecture du plugin "Formation Flying" de
 x-plane.org. X-Plane ne fait alors plus que dessiner l'avion là où on le met ;
 toute la dynamique (suivi, lissage, transitions) est calculée ici.

 ANTI-JITTER
 -----------
 On ne "snappe" jamais l'IA sur sa position cible. À chaque frame :
   1. on calcule la position cible = position du joueur + offset de formation
      tourné par le cap du joueur ;
   2. on ajoute un feed-forward de vitesse (cible += V_joueur * tau) qui
      annule le retard stationnaire du filtre — sans lui, l'escorte traînerait
      de V*tau mètres derrière sa place en croisière ;
   3. on filtre la position réelle de l'IA vers cette cible avec un lissage
      exponentiel dont le coefficient dépend du delta-time RÉEL de la frame :
          alpha = 1 - exp(-dt / tau)
      => comportement identique à 30 ou 60 fps (aucun délai codé "en frames").

 DATAREFS UTILISÉES
 ------------------
 LECTURE (avion joueur — fonctionne avec n'importe quel appareil) :
   sim/flightmodel/position/local_x,_y,_z   position OpenGL (m, double)
   sim/flightmodel/position/local_vx,vy,vz  vitesse sol (m/s)
   sim/flightmodel/position/psi,theta,phi   cap / assiette / roulis (deg)
   sim/flightmodel/failures/onground_any    1 = au moins une roue au sol
   sim/cockpit2/controls/gear_handle_down   position manette de train
   sim/operation/misc/frame_rate_period     dt réel de la frame (s)
   sim/time/paused                          1 = simu en pause

 ÉCRITURE (avions IA n = 1..19, datarefs "multiplayer" classiques qui pilotent
 les avions IA quand override_planepath[n] = 1) :
   sim/operation/override/override_planepath[n]  1 = notre script pilote l'IA n
   sim/multiplayer/position/planeN_x,_y,_z       position OpenGL de l'IA
   sim/multiplayer/position/planeN_psi,_the,_phi cap / assiette / roulis
   sim/multiplayer/position/planeN_v_x,_v_y,_v_z vitesses (pour TCAS/sons)
   sim/multiplayer/position/planeN_gear_deploy   sortie de train (0..1, x10)

 SDK natif (optionnel, via LuaJIT FFI) :
   XPLMCreateProbe / XPLMProbeTerrainXYZ : sonde terrain pour poser les IA
   exactement sur le tarmac pendant le roulage même si l'aéroport est en
   pente. Si le FFI échoue, repli automatique sur l'altitude du joueur
   (correct sur un parking plat).

 PHASES
 ------
 SOL   : formation "trail" lâche derrière le joueur (lissage lent = 2.5 s),
         altitude collée au terrain (sonde) + garde au sol calibrée à
         l'activation, cap = direction réelle de déplacement de l'IA.
 TRANSITION : dès que le joueur quitte le sol, un facteur de mélange "blend"
         (lui-même lissé, tau 4 s) fait glisser continûment les offsets
         trail->V, l'altitude terrain->altitude joueur et l'assiette
         0 -> assiette joueur. Aucun saut : tout est interpolé, et l'altitude
         reste bornée au-dessus du terrain tant que blend < 1.
 VOL   : formation en V / échelon de part et d'autre du joueur (lissage
         rapide = 0.35 s), distances latérale/verticale/longitudinale
         réglables dans la fenêtre.

 COMMANDES CRÉÉES (assignables à un joystick/clavier) :
   FlyWithLua/FighterEscort/toggle_escort   activer / désactiver l'escorte
   FlyWithLua/FighterEscort/toggle_window   afficher / masquer la fenêtre
   FlyWithLua/FighterEscort/snap            re-placer les IA en formation
================================================================================
]]

-- ============================================================================
-- CONFIGURATION (modifiable en jeu via la fenêtre ; sauvegardée sur disque)
-- ============================================================================
local cfg = {
    ai_index      = { 1, 2 },  -- index des 2 avions IA pilotés (1..19)
    lat_dist      = 30.0,      -- [vol] écart latéral de chaque ailier (m)
    vert_dist     = 0.0,       -- [vol] écart vertical (m, + = au-dessus)
    back_dist     = 20.0,      -- [vol] retrait longitudinal (m, derrière)
    taxi_gap      = 45.0,      -- [sol] espacement en file pendant le taxi (m)
    taxi_lat      = 7.0,       -- [sol] décalage latéral alterné au taxi (m)
    enabled       = false,     -- escorte active ?
    debug         = false,     -- overlay debug à l'écran
}

-- Constantes de dynamique (pas exposées dans l'UI, valeurs éprouvées)
local TAU_POS_GND   = 2.5    -- lissage position au sol (s) — formation lâche
local TAU_POS_AIR   = 0.35   -- lissage position en vol (s) — formation tenue
local TAU_ATT       = 0.30   -- lissage cap/assiette/roulis (s)
local TAU_BLEND     = 4.0    -- durée de la transition sol <-> vol (s)
local SNAP_DIST     = 500.0  -- au-delà de cette erreur (m) on téléporte l'IA
local DT_MIN, DT_MAX = 1/200, 0.2  -- bornes de sécurité sur le delta-time

local CFG_FILE = SCRIPT_DIRECTORY .. "FighterEscort.cfg"

-- ============================================================================
-- DATAREFS AVION JOUEUR (liées en variables globales, mises à jour par FWL)
-- ============================================================================
dataref("usr_x",   "sim/flightmodel/position/local_x",  "readonly")
dataref("usr_y",   "sim/flightmodel/position/local_y",  "readonly")
dataref("usr_z",   "sim/flightmodel/position/local_z",  "readonly")
dataref("usr_vx",  "sim/flightmodel/position/local_vx", "readonly")
dataref("usr_vy",  "sim/flightmodel/position/local_vy", "readonly")
dataref("usr_vz",  "sim/flightmodel/position/local_vz", "readonly")
dataref("usr_psi", "sim/flightmodel/position/psi",      "readonly")
dataref("usr_the", "sim/flightmodel/position/theta",    "readonly")
dataref("usr_phi", "sim/flightmodel/position/phi",      "readonly")
dataref("usr_gnd", "sim/flightmodel/failures/onground_any", "readonly")
dataref("usr_gear","sim/cockpit2/controls/gear_handle_down", "readonly")
dataref("sim_dt",  "sim/operation/misc/frame_rate_period",   "readonly")
dataref("sim_paused", "sim/time/paused", "readonly")

-- Tableau d'override : override_planepath[n] = 1 coupe l'ATC/FMS de l'IA n
local ovr_planepath = dataref_table("sim/operation/override/override_planepath")

-- ============================================================================
-- SONDE TERRAIN (SDK natif via FFI, avec repli silencieux si indisponible)
-- ============================================================================
local ffi_ok, ffi = pcall(require, "ffi")
local XPLM, probe_ref, probe_info = nil, nil, nil
if ffi_ok then
    pcall(ffi.cdef, [[
        typedef void *XPLMProbeRef;
        typedef struct {
            int   structSize;
            float locationX; float locationY; float locationZ;
            float normalX;   float normalY;   float normalZ;
            float velocityX; float velocityY; float velocityZ;
            int   is_wet;
        } XPLMProbeInfo_t;
        XPLMProbeRef XPLMCreateProbe(int inProbeType);
        int XPLMProbeTerrainXYZ(XPLMProbeRef inProbe,
                                float inX, float inY, float inZ,
                                XPLMProbeInfo_t *outInfo);
    ]])
    local ok, lib = pcall(function()
        if     SYSTEM == "IBM" then return ffi.load("XPLM_64")
        elseif SYSTEM == "LIN" then return ffi.load("Resources/plugins/XPLM_64.so")
        else return ffi.load("Resources/plugins/XPLM.framework/XPLM") -- macOS
        end
    end)
    if ok and lib then
        XPLM = lib
        local ok2, ref = pcall(XPLM.XPLMCreateProbe, 0) -- 0 = xplm_ProbeY
        if ok2 then
            probe_ref  = ref
            probe_info = ffi.new("XPLMProbeInfo_t")
        end
    end
end

-- Altitude du terrain sous (x,z). 'fallback' est renvoyé si la sonde échoue.
local function terrain_y(x, y, z, fallback)
    if probe_ref then
        probe_info.structSize = ffi.sizeof("XPLMProbeInfo_t")
        -- On sonde depuis 150 m au-dessus pour être sûr de partir hors sol.
        if XPLM.XPLMProbeTerrainXYZ(probe_ref, x, y + 150.0, z, probe_info) == 0 then
            return probe_info.locationY
        end
    end
    return fallback
end

-- ============================================================================
-- OUTILS MATH
-- ============================================================================
local function clamp(v, lo, hi) return math.max(lo, math.min(hi, v)) end
local function lerp(a, b, t)    return a + (b - a) * t end

-- Coefficient de lissage exponentiel indépendant du frame-rate.
local function smooth_alpha(dt, tau) return 1.0 - math.exp(-dt / tau) end

-- Plus court chemin angulaire (deg), pour lisser un cap sans faire un 350°.
local function ang_diff(target, current)
    return ((target - current + 540.0) % 360.0) - 180.0
end

-- ============================================================================
-- ÉTAT INTERNE DES 2 ESCORTEURS
-- ============================================================================
-- side = -1 : ailier gauche ; side = +1 : ailier droit
local escorts = {
    { side = -1, rank = 1 },
    { side =  1, rank = 2 },
}
for _, e in ipairs(escorts) do
    e.init   = false  -- état lissé initialisé ?
    e.sx, e.sy, e.sz = 0, 0, 0        -- position lissée (écrite dans le simu)
    e.spsi, e.sthe, e.sphi = 0, 0, 0  -- attitude lissée
    e.px, e.py, e.pz = 0, 0, 0        -- position frame précédente (vitesses)
    e.gear_h = 2.0                    -- garde au sol calibrée à l'activation
    e.err    = 0                      -- erreur cible/réel (debug)
    e.drefs  = nil                    -- noms de datarefs de l'IA (cache)
    e.gear_tbl = nil                  -- dataref_table du train de l'IA
end

local blend = 0.0        -- 0 = phase sol, 1 = phase vol (lissé)
local status_msg = "Escorte inactive"

-- Construit et met en cache les noms de datarefs de l'avion IA d'index n.
local function build_drefs(n)
    local p = "sim/multiplayer/position/plane" .. n
    return {
        x = p.."_x",   y = p.."_y",   z = p.."_z",
        psi = p.."_psi", the = p.."_the", phi = p.."_phi",
        vx = p.."_v_x", vy = p.."_v_y", vz = p.."_v_z",
        gear = p.."_gear_deploy",
    }
end

local function refresh_drefs(e)
    e.drefs    = build_drefs(cfg.ai_index[e.rank])
    e.gear_tbl = dataref_table(e.drefs.gear)
end

-- Un avion IA non chargé renvoie une position (0,0,0) : on le détecte ainsi.
local function ai_loaded(e)
    if not e.drefs then refresh_drefs(e) end
    local x, z = get(e.drefs.x), get(e.drefs.z)
    return not (x == 0 and z == 0)
end

-- ============================================================================
-- CALCUL DE LA POSITION CIBLE D'UN ESCORTEUR
-- ============================================================================
-- Offsets en repère avion joueur (fwd = vers l'avant, right = vers la droite,
-- up = vers le haut), puis rotation par le cap. Repère OpenGL X-Plane :
-- X = est, Y = haut, Z = sud  =>  forward = (sin psi, 0, -cos psi),
--                                 right   = (cos psi, 0,  sin psi).
local function formation_target(e, tau_pos)
    -- Offsets phase SOL : file indienne décalée derrière le joueur.
    local g_fwd   = -cfg.taxi_gap * e.rank
    local g_right = e.side * cfg.taxi_lat
    -- Offsets phase VOL : V / échelon de part et d'autre.
    local a_fwd   = -cfg.back_dist
    local a_right = e.side * cfg.lat_dist
    -- Mélange continu sol <-> vol (aucun saut à la rotation).
    local fwd   = lerp(g_fwd,   a_fwd,   blend)
    local right = lerp(g_right, a_right, blend)

    local psi_r = math.rad(usr_psi)
    local sinp, cosp = math.sin(psi_r), math.cos(psi_r)
    local tx = usr_x + fwd * sinp + right * cosp
    local tz = usr_z - fwd * cosp + right * sinp

    -- Altitude : au sol on colle au terrain (sonde), en vol on suit le joueur.
    local ty
    if blend >= 0.999 then
        ty = usr_y + cfg.vert_dist
    else
        local ter   = terrain_y(tx, usr_y, tz, usr_y - e.gear_h)
        local ty_g  = ter + e.gear_h
        local ty_a  = usr_y + cfg.vert_dist
        ty = lerp(ty_g, ty_a, blend)
        ty = math.max(ty, ter + 0.8 * e.gear_h) -- jamais dans le sol
    end

    -- Feed-forward : annule le retard stationnaire du filtre (voir en-tête).
    tx = tx + usr_vx * tau_pos
    tz = tz + usr_vz * tau_pos
    ty = ty + usr_vy * tau_pos * blend -- pas de FF vertical au sol

    return tx, ty, tz
end

-- ============================================================================
-- ACTIVATION / DÉSACTIVATION / SNAP
-- ============================================================================
-- Téléporte un escorteur pile sur sa position de formation (état lissé remis
-- à zéro dessus => aucune "course de rattrapage" au premier frame).
local function snap_escort(e)
    local tau = lerp(TAU_POS_GND, TAU_POS_AIR, blend)
    local tx, ty, tz = formation_target(e, tau)
    e.sx, e.sy, e.sz = tx, ty, tz
    e.px, e.py, e.pz = tx, ty, tz
    e.spsi, e.sthe, e.sphi = usr_psi, 0, 0
    e.init = true
end

function escort_snap_all()
    if not cfg.enabled then return end
    for _, e in ipairs(escorts) do
        if ai_loaded(e) then snap_escort(e) end
    end
end

-- Calibre la garde au sol de l'IA : hauteur entre son point d'origine (posé
-- au parking par X-Plane) et le terrain — évite train enterré ou avion qui
-- flotte, quel que soit le modèle 3D choisi comme escorteur.
local function calibrate_gear_height(e)
    local ax, ay, az = get(e.drefs.x), get(e.drefs.y), get(e.drefs.z)
    local ter = terrain_y(ax, ay, az, nil)
    if ter then
        local h = ay - ter
        if h > 0.3 and h < 8.0 then e.gear_h = h return end
    end
    e.gear_h = 2.0 -- valeur raisonnable par défaut
end

local function enable_escort()
    blend = (usr_gnd == 1) and 0.0 or 1.0
    local ok_count = 0
    for _, e in ipairs(escorts) do
        refresh_drefs(e)
        if ai_loaded(e) then
            calibrate_gear_height(e)
            ovr_planepath[cfg.ai_index[e.rank]] = 1 -- on prend la main sur l'IA
            snap_escort(e)
            ok_count = ok_count + 1
        else
            e.init = false
        end
    end
    if ok_count == 0 then
        status_msg = "ERREUR : aucun avion IA charge (verifier les settings X-Plane)"
        cfg.enabled = false
    else
        status_msg = string.format("Escorte active (%d/2 IA)", ok_count)
        cfg.enabled = true
    end
end

local function disable_escort()
    for _, e in ipairs(escorts) do
        ovr_planepath[cfg.ai_index[e.rank]] = 0 -- l'ATC interne reprend la main
        e.init = false
    end
    cfg.enabled = false
    status_msg = "Escorte inactive"
end

function escort_toggle()
    if cfg.enabled then disable_escort() else enable_escort() end
end

-- ============================================================================
-- BOUCLE PRINCIPALE (chaque frame ; tout est basé sur le dt réel)
-- ============================================================================
function fighter_escort_frame()
    if not cfg.enabled then return end
    if sim_paused == 1 then return end

    local dt = clamp(sim_dt, DT_MIN, DT_MAX)

    -- Facteur de phase sol/vol, lui-même lissé pour une transition douce.
    local blend_target = (usr_gnd == 1) and 0.0 or 1.0
    blend = blend + (blend_target - blend) * smooth_alpha(dt, TAU_BLEND)

    local tau_pos = lerp(TAU_POS_GND, TAU_POS_AIR, blend)
    local a_pos   = smooth_alpha(dt, tau_pos)
    local a_att   = smooth_alpha(dt, TAU_ATT)

    for _, e in ipairs(escorts) do
        if e.init and ai_loaded(e) then
            local tx, ty, tz = formation_target(e, tau_pos)

            -- Sécurité : joueur repositionné (menu Location, etc.) => snap.
            local dx, dy, dz = tx - e.sx, ty - e.sy, tz - e.sz
            e.err = math.sqrt(dx*dx + dy*dy + dz*dz)
            if e.err > SNAP_DIST then snap_escort(e) end

            -- 1) Lissage exponentiel de la position (anti-jitter).
            e.sx = e.sx + (tx - e.sx) * a_pos
            e.sy = e.sy + (ty - e.sy) * a_pos
            e.sz = e.sz + (tz - e.sz) * a_pos

            -- 2) Vitesses réelles de l'IA (pour TCAS, sons, réseau).
            local vx = (e.sx - e.px) / dt
            local vy = (e.sy - e.py) / dt
            local vz = (e.sz - e.pz) / dt

            -- 3) Attitude cible.
            --    Sol : cap = direction réelle de déplacement (l'IA "roule"
            --    naturellement dans les virages de taxiway) ; assiette nulle.
            --    Vol : cap/assiette/roulis du joueur (il vire => ils virent).
            local spd2d = math.sqrt(vx*vx + vz*vz)
            local psi_gnd = (spd2d > 1.0) and math.deg(math.atan2(vx, -vz)) or e.spsi
            local psi_tgt = e.spsi + ang_diff(lerp(psi_gnd, usr_psi, blend), e.spsi)
            local the_tgt = usr_the * blend
            local phi_tgt = usr_phi * blend

            e.spsi = e.spsi + ang_diff(psi_tgt, e.spsi) * a_att
            e.sthe = e.sthe + (the_tgt - e.sthe) * a_att
            e.sphi = e.sphi + (phi_tgt - e.sphi) * a_att

            -- 4) Écriture dans le simulateur (l'override rend ces datarefs
            --    pilotables ; X-Plane ne fait plus que dessiner l'avion).
            set(e.drefs.x,   e.sx)
            set(e.drefs.y,   e.sy)
            set(e.drefs.z,   e.sz)
            set(e.drefs.psi, e.spsi % 360.0)
            set(e.drefs.the, e.sthe)
            set(e.drefs.phi, e.sphi)
            set(e.drefs.vx,  vx)
            set(e.drefs.vy,  vy)
            set(e.drefs.vz,  vz)

            -- 5) Train : sorti au sol / pendant la transition, sinon il suit
            --    la manette de train du joueur (rentre peu après le décollage).
            local gear_cmd = (blend < 0.7 or usr_gear == 1) and 1.0 or 0.0
            if e.gear_tbl then
                for i = 0, 9 do
                    local g = e.gear_tbl[i]
                    e.gear_tbl[i] = g + (gear_cmd - g) * smooth_alpha(dt, 1.5)
                end
            end

            e.px, e.py, e.pz = e.sx, e.sy, e.sz
        end
    end
end

do_every_frame("fighter_escort_frame()")

-- ============================================================================
-- OVERLAY DEBUG (positions calculées, offsets, erreurs de formation)
-- ============================================================================
function fighter_escort_draw()
    if not cfg.debug then return end
    local y = SCREEN_HIGHT - 80
    draw_string(20, y, string.format(
        "[FighterEscort] blend=%.2f  sol=%d  joueur x=%.0f y=%.0f z=%.0f psi=%.0f",
        blend, usr_gnd, usr_x, usr_y, usr_z, usr_psi), 0.2, 1.0, 0.2)
    for _, e in ipairs(escorts) do
        y = y - 18
        if e.init then
            draw_string(20, y, string.format(
                "  IA#%d (plane%d)  pos=%.0f/%.0f/%.0f  psi=%.0f  err=%.1fm  gearH=%.1f",
                e.rank, cfg.ai_index[e.rank], e.sx, e.sy, e.sz,
                e.spsi % 360, e.err, e.gear_h), 0.2, 1.0, 0.2)
        else
            draw_string(20, y, string.format(
                "  IA#%d (plane%d)  NON PILOTE (IA chargee ? escorte active ?)",
                e.rank, cfg.ai_index[e.rank]), 1.0, 0.4, 0.2)
        end
    end
end

do_every_draw("fighter_escort_draw()")

-- ============================================================================
-- SAUVEGARDE / CHARGEMENT DES RÉGLAGES
-- ============================================================================
local function save_cfg()
    local f = io.open(CFG_FILE, "w")
    if not f then return end
    f:write(string.format("ai1=%d\nai2=%d\nlat=%.1f\nvert=%.1f\nback=%.1f\ntaxi_gap=%.1f\ntaxi_lat=%.1f\n",
        cfg.ai_index[1], cfg.ai_index[2], cfg.lat_dist, cfg.vert_dist,
        cfg.back_dist, cfg.taxi_gap, cfg.taxi_lat))
    f:close()
end

local function load_cfg()
    local f = io.open(CFG_FILE, "r")
    if not f then return end
    for line in f:lines() do
        local k, v = line:match("^(%w+)=([%-%d%.]+)$")
        if k and v then
            v = tonumber(v)
            if     k == "ai1"      then cfg.ai_index[1] = clamp(math.floor(v), 1, 19)
            elseif k == "ai2"      then cfg.ai_index[2] = clamp(math.floor(v), 1, 19)
            elseif k == "lat"      then cfg.lat_dist  = clamp(v, 10, 200)
            elseif k == "vert"     then cfg.vert_dist = clamp(v, -50, 50)
            elseif k == "back"     then cfg.back_dist = clamp(v, 0, 200)
            elseif k == "taxi_gap" then cfg.taxi_gap  = clamp(v, 20, 150)
            elseif k == "taxi_lat" then cfg.taxi_lat  = clamp(v, 0, 30)
            end
        end
    end
    f:close()
end
load_cfg()

-- ============================================================================
-- FENÊTRE DE RÉGLAGES (imgui / FlyWithLua NG)
-- ============================================================================
local wnd = nil

function escort_build_ui()
    imgui.TextUnformatted("Statut : " .. status_msg)
    imgui.Separator()

    -- Activation
    local changed, val = imgui.Checkbox("Escorte active", cfg.enabled)
    if changed then
        if val then enable_escort() else disable_escort() end
    end
    imgui.SameLine()
    if imgui.Button("Re-placer en formation") then escort_snap_all() end

    imgui.Separator()
    imgui.TextUnformatted("Avions IA (index 1-19, cf. settings Air Traffic) :")
    for r = 1, 2 do
        local ch, v = imgui.SliderInt(
            string.format("IA n.%d (ailier %s)", r, r == 1 and "gauche" or "droit"),
            cfg.ai_index[r], 1, 19)
        if ch then
            -- On relâche l'ancien avion avant de prendre le nouveau.
            if cfg.enabled then ovr_planepath[cfg.ai_index[r]] = 0 end
            cfg.ai_index[r] = v
            refresh_drefs(escorts[r])
            if cfg.enabled and ai_loaded(escorts[r]) then
                calibrate_gear_height(escorts[r])
                ovr_planepath[v] = 1
                snap_escort(escorts[r])
            end
            save_cfg()
        end
        imgui.SameLine()
        imgui.TextUnformatted(ai_loaded(escorts[r]) and "[chargee]" or "[ABSENTE]")
    end

    imgui.Separator()
    imgui.TextUnformatted("Formation en vol :")
    local ch1, v1 = imgui.SliderFloat("Ecart lateral (m)",    cfg.lat_dist,  10, 200, "%.0f")
    if ch1 then cfg.lat_dist = v1;  save_cfg() end
    local ch2, v2 = imgui.SliderFloat("Ecart vertical (m)",   cfg.vert_dist, -50, 50, "%.0f")
    if ch2 then cfg.vert_dist = v2; save_cfg() end
    local ch3, v3 = imgui.SliderFloat("Retrait arriere (m)",  cfg.back_dist,  0, 200, "%.0f")
    if ch3 then cfg.back_dist = v3; save_cfg() end

    imgui.TextUnformatted("Roulage :")
    local ch4, v4 = imgui.SliderFloat("Espacement taxi (m)",  cfg.taxi_gap,  20, 150, "%.0f")
    if ch4 then cfg.taxi_gap = v4;  save_cfg() end

    imgui.Separator()
    local chd, vd = imgui.Checkbox("Mode debug (overlay ecran)", cfg.debug)
    if chd then cfg.debug = vd end
    imgui.TextUnformatted(probe_ref and "Sonde terrain SDK : OK"
                                    or  "Sonde terrain SDK : indisponible (repli altitude joueur)")
end

function escort_wnd_closed()
    wnd = nil
end

function escort_toggle_window()
    if wnd then
        float_wnd_destroy(wnd)
        wnd = nil
    else
        wnd = float_wnd_create(430, 360, 1, true)
        float_wnd_set_title(wnd, "Fighter Escort")
        float_wnd_set_imgui_builder(wnd, "escort_build_ui")
        float_wnd_set_onclose(wnd, "escort_wnd_closed")
    end
end

-- ============================================================================
-- COMMANDES + ENTRÉE DE MENU
-- ============================================================================
create_command("FlyWithLua/FighterEscort/toggle_escort",
    "Fighter Escort : activer/desactiver l'escorte", "escort_toggle()", "", "")
create_command("FlyWithLua/FighterEscort/toggle_window",
    "Fighter Escort : afficher/masquer la fenetre", "escort_toggle_window()", "", "")
create_command("FlyWithLua/FighterEscort/snap",
    "Fighter Escort : re-placer les IA en formation", "escort_snap_all()", "", "")

-- Accessible aussi via Plugins > FlyWithLua > FlyWithLua Macros
add_macro("Fighter Escort : fenetre de reglages",
          "escort_toggle_window()", "escort_toggle_window()", "deactivate")

-- Ouvre la fenêtre au premier chargement pour que le script soit découvrable.
escort_toggle_window()
