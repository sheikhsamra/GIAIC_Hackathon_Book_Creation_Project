import React, { useEffect } from 'react';
import { useLocation } from '@docusaurus/router';
import TranslatorButton from '../TranslatorButton';

const TranslatorInjector = () => {
  const location = useLocation();

  useEffect(() => {
    // This component will be injected in the navbar
    // The actual injection happens via the theme
  }, [location]);

  return <TranslatorButton />;
};

export default TranslatorInjector;