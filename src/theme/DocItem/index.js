import React from 'react';
import DocItem from '@theme-original/DocItem';
import TranslatorButton from '@site/src/components/TranslatorButton';

export default function DocItemWithTranslator(props) {
  return (
    <>
      <div style={{ display: 'flex', justifyContent: 'flex-end', marginBottom: '1rem' }}>
        <TranslatorButton />
      </div>
      <DocItem {...props} />
    </>
  );
}