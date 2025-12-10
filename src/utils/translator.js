// Translator functionality for Docusaurus book
class BookTranslator {
  constructor() {
    this.isUrdu = false;
    this.scrollPosition = 0;
    this.init();
  }

  init() {
    // Wait for DOM to be ready
    if (document.readyState === 'loading') {
      document.addEventListener('DOMContentLoaded', () => this.setupTranslator());
    } else {
      this.setupTranslator();
    }
  }

  setupTranslator() {
    const toggleButton = document.getElementById('translator-toggle');
    if (!toggleButton) return;

    // Load saved language preference
    this.loadSavedPreference();

    // Add click event listener
    toggleButton.addEventListener('click', () => this.toggleLanguage());

    // Update button text based on current language
    this.updateButtonText(toggleButton);
  }

  loadSavedPreference() {
    const savedLanguage = localStorage.getItem('bookLanguage');
    if (savedLanguage === 'urdu') {
      this.isUrdu = true;
      this.applyUrduStyles();
    }
  }

  toggleLanguage() {
    this.isUrdu = !this.isUrdu;

    // Save preference
    localStorage.setItem('bookLanguage', this.isUrdu ? 'urdu' : 'english');

    // Apply or remove Urdu styles
    if (this.isUrdu) {
      this.applyUrduStyles();
    } else {
      this.removeUrduStyles();
    }

    // Update button text
    const toggleButton = document.getElementById('translator-toggle');
    if (toggleButton) {
      this.updateButtonText(toggleButton);
    }

    // Preserve scroll position
    this.preserveScrollPosition();
  }

  applyUrduStyles() {
    document.body.classList.add('urdu-content');
    document.body.dir = 'rtl';
  }

  removeUrduStyles() {
    document.body.classList.remove('urdu-content');
    document.body.dir = 'ltr';
  }

  updateButtonText(button) {
    if (this.isUrdu) {
      button.innerHTML = '🇺🇸 English';
      button.style.backgroundColor = '#dc2626';
    } else {
      button.innerHTML = '🇵🇰 Urdu';
      button.style.backgroundColor = '#2563eb';
    }
  }

  preserveScrollPosition() {
    // Store current scroll position
    this.scrollPosition = window.scrollY;

    // Restore scroll position after a brief moment
    setTimeout(() => {
      window.scrollTo(0, this.scrollPosition);
    }, 10);
  }
}

// Initialize the translator when the page loads
document.addEventListener('DOMContentLoaded', () => {
  new BookTranslator();
});

// Also initialize if DOM is already loaded
if (document.readyState === 'complete') {
  new BookTranslator();
}