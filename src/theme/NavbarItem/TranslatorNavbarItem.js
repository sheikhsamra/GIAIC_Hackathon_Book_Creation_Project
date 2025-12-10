import React from 'react';
import NavbarItem from '@theme-original/NavbarItem';
import { useLocation } from '@docusaurus/router';
import TranslatorButton from '../../components/TranslatorButton';

const TranslatorNavbarItem = (props) => {
  const location = useLocation();

  return (
    <div className="navbar__item translator-navbar-item">
      <TranslatorButton />
    </div>
  );
};

export default TranslatorNavbarItem;