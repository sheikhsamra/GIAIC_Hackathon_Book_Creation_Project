// Translator functionality for Docusaurus book
class BookTranslator {
  constructor() {
    this.isUrdu = false;
    this.scrollPosition = 0;
    this.init();
  }

  init() {
    // Wait for DOM to be ready and also monitor for dynamic content
    if (document.readyState === 'loading') {
      document.addEventListener('DOMContentLoaded', () => this.setupTranslator());
    } else {
      this.setupTranslator();
    }

    // For SPAs like Docusaurus, also listen for route changes
    this.observeRouteChanges();
  }

  observeRouteChanges() {
    // Monitor for URL changes which indicate page navigation in SPAs
    let currentPath = location.pathname;
    setInterval(() => {
      if (location.pathname !== currentPath) {
        currentPath = location.pathname;
        // Re-setup translator after route change
        setTimeout(() => this.setupTranslator(), 100);
      }
    }, 500);
  }

  setupTranslator() {
    // Use a more reliable way to get the button with retry logic
    let attempts = 0;
    const maxAttempts = 20; // Try for up to 10 seconds

    const trySetup = () => {
      const toggleButton = document.getElementById('translator-toggle');
      attempts++;

      if (toggleButton) {
        // Load saved language preference
        this.loadSavedPreference();

        // Remove any existing event listeners to avoid duplicates
        toggleButton.replaceWith(toggleButton.cloneNode(true));
        const newButton = document.getElementById('translator-toggle');

        // Add click event listener to the new button
        newButton.addEventListener('click', () => this.toggleLanguage());

        // Update button text based on current language
        this.updateButtonText(newButton);
      } else if (attempts < maxAttempts) {
        // Retry after a short delay
        setTimeout(trySetup, 500);
      }
    };

    trySetup();
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
    document.body.setAttribute('dir', 'rtl');
  }

  removeUrduStyles() {
    document.body.classList.remove('urdu-content');
    document.body.setAttribute('dir', 'ltr');
  }

  updateButtonText(button) {
    if (button) {
      if (this.isUrdu) {
        button.innerHTML = '🇺🇸 English';
        button.style.backgroundColor = '#dc2626';
      } else {
        button.innerHTML = '🇵🇰 Urdu';
        button.style.backgroundColor = '#2563eb';
      }
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
if (document.readyState === 'loading') {
  document.addEventListener('DOMContentLoaded', () => {
    new BookTranslator();
  });
} else {
  // Add a slight delay to ensure DOM is fully ready
  setTimeout(() => {
    new BookTranslator();
  }, 100);
}

// Handle cases where script loads after DOM
if (document.readyState === 'complete') {
  setTimeout(() => {
    new BookTranslator();
  }, 100);
}