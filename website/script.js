// Sticky header shadow on scroll
const header = document.getElementById('header');
window.addEventListener('scroll', () => {
    header.classList.toggle('scrolled', window.scrollY > 10);
}, { passive: true });

// Mobile burger menu
const burger  = document.getElementById('burger');
const navList = document.getElementById('navList');

burger.addEventListener('click', () => {
    const isOpen = navList.classList.toggle('open');
    burger.setAttribute('aria-expanded', isOpen);
    burger.setAttribute('aria-label', isOpen ? 'Fermer le menu' : 'Ouvrir le menu');
});

// Close menu on nav link click
document.querySelectorAll('.nav__link').forEach(link => {
    link.addEventListener('click', () => {
        navList.classList.remove('open');
        burger.setAttribute('aria-expanded', 'false');
    });
});

// Close menu on outside click
document.addEventListener('click', (e) => {
    if (!header.contains(e.target) && navList.classList.contains('open')) {
        navList.classList.remove('open');
        burger.setAttribute('aria-expanded', 'false');
    }
});

// Contact form — feedback visuel à la soumission
const form = document.getElementById('contactForm');
form.addEventListener('submit', (e) => {
    e.preventDefault();

    const btn = form.querySelector('button[type="submit"]');
    const original = btn.textContent;

    btn.textContent = 'Message envoyé ✓';
    btn.disabled = true;
    btn.style.cssText = 'background:#27AE60; border-color:#27AE60; cursor:default;';

    form.reset();

    setTimeout(() => {
        btn.textContent = original;
        btn.disabled = false;
        btn.style.cssText = '';
    }, 5000);
});
