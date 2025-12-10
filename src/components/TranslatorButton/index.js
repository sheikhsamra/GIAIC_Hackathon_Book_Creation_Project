import React, { useState, useEffect, useRef } from 'react';
import { useLocation } from '@docusaurus/router';
import { useActiveDocContext } from '@docusaurus/plugin-content-docs/client';

const TranslatorButton = () => {
  const location = useLocation();
  const { activeDoc } = useActiveDocContext();
  // State to track user preference
  const [userPrefersUrdu, setUserPrefersUrdu] = useState(false);
  const [isLoading, setIsLoading] = useState(false);
  const [statusMessage, setStatusMessage] = useState('');
  const originalContentRef = useRef(null);

  // Extract module from the current path
  const getCurrentModule = () => {
    if (activeDoc && activeDoc.id) {
      // Use the active doc context to get the current doc ID
      // For urdu-translation pages, we need to extract the base module
      if (activeDoc.id.includes('urdu-translation')) {
        return activeDoc.id.replace('/urdu-translation', '').split('/')[0];
      }
      return activeDoc.id.split('/')[0]; // Get the base module (e.g., 'module-1')
    }

    // Fallback: extract module from the pathname
    const match = location.pathname.match(/\/docs\/(module-\d+|capstone)/);
    return match ? match[1] : null;
  };

  // Initialize user preference based on localStorage
  useEffect(() => {
    const savedLanguage = localStorage.getItem('book-language');
    if (savedLanguage === 'ur') {
      setUserPrefersUrdu(true);
    }
  }, []);

  const toggleTranslation = async () => {
    const module = getCurrentModule();
    if (!module) {
      setStatusMessage('Cannot determine current module');
      return;
    }

    setIsLoading(true);
    setStatusMessage(userPrefersUrdu ? 'Switching to English…' : 'Loading Urdu translation…');

    try {
      if (userPrefersUrdu) {
        // Switch back to English
        restoreEnglishContent();
        setUserPrefersUrdu(false);
        setStatusMessage('Back to English');
      } else {
        // Fetch and show Urdu translation
        await loadUrduTranslation(module);
        setUserPrefersUrdu(true);
        setStatusMessage('Urdu translation loaded');
      }

      // Save the language preference
      localStorage.setItem('book-language', userPrefersUrdu ? 'en' : 'ur');
    } catch (error) {
      console.error('Error with translation:', error);
      setStatusMessage(`Error: ${error.message}`);
    } finally {
      setIsLoading(false);
      setTimeout(() => setStatusMessage(''), 2000);
    }
  };

  const restoreEnglishContent = () => {
    const docContentElement = document.querySelector('article.markdown');
    if (docContentElement && originalContentRef.current) {
      docContentElement.innerHTML = originalContentRef.current;

      // Remove Urdu styling classes from the document body
      document.body.classList.remove('urdu-content');
      document.body.setAttribute('dir', 'ltr');
    }
  };

  const loadUrduTranslation = async (module) => {
    // Store the original English content to restore later
    const docContentElement = document.querySelector('article.markdown');
    if (docContentElement) {
      originalContentRef.current = docContentElement.innerHTML;
    }

    // Fetch the Urdu translation page for the current module
    // In Docusaurus, the urdu-translation.md file should be built as a page at /docs/[module]/urdu-translation
    const response = await fetch(`/docs/${module}/urdu-translation`);

    if (!response.ok) {
      throw new Error(`Urdu translation page not found for ${module}`);
    }

    const htmlText = await response.text();

    // Parse the HTML to extract the content
    const parser = new DOMParser();
    const doc = parser.parseFromString(htmlText, 'text/html');

    // Look for the main content area in the parsed document
    const contentElement = doc.querySelector('article.markdown') || doc.querySelector('main article') || doc.querySelector('.markdown');

    if (!contentElement) {
      throw new Error('Could not extract content from Urdu translation page');
    }

    // Update the current page's content with the Urdu content
    const targetElement = document.querySelector('article.markdown');
    if (targetElement) {
      targetElement.innerHTML = contentElement.innerHTML;

      // Apply Urdu styling classes to the document body
      document.body.classList.add('urdu-content');
      document.body.setAttribute('dir', 'rtl');
    }
  };

  return (
    <div className="translator-container">
      <button
        onClick={toggleTranslation}
        className={`translator-button ${userPrefersUrdu ? 'urdu-mode' : 'english-mode'} ${isLoading ? 'loading' : ''}`}
        aria-pressed={userPrefersUrdu}
        aria-label={userPrefersUrdu ? 'Show in English' : 'Translate to Urdu'}
        disabled={isLoading}
      >
        {isLoading ? (
          <span className="loading-spinner">🔄</span>
        ) : (
          <span>{userPrefersUrdu ? '🇺🇸 English' : '🇵🇰 Urdu'}</span>
        )}
      </button>

      {statusMessage && (
        <div
          className="translator-status"
          aria-live="polite"
          role="status"
        >
          {statusMessage}
        </div>
      )}
    </div>
  );
};

export default TranslatorButton;