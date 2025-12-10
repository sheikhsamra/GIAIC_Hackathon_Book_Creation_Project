import ExecutionEnvironment from '@docusaurus/ExecutionEnvironment';
import TranslatorButton from './components/TranslatorButton';

if (ExecutionEnvironment.canUseDOM) {
  // Wait for the DOM to be loaded
  window.addEventListener('load', () => {
    const container = document.getElementById('translator-button-container');
    if (container) {
      // Create a wrapper element for React rendering
      const wrapper = document.createElement('div');
      container.appendChild(wrapper);

      // Render the TranslatorButton component
      const React = require('react');
      const ReactDOM = require('react-dom');

      ReactDOM.render(React.createElement(TranslatorButton), wrapper);
    }
  });
}