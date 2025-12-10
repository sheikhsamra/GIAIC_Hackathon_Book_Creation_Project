import React, { useState } from 'react';
import clsx from 'clsx';
import styles from './CodeExample.module.css';
import useBaseUrl from '@docusaurus/useBaseUrl';

const CodeExample = ({
  code,
  language = 'python',
  title = 'Code Example',
  description,
  safetyNotes,
  environment,
  dependencies
}) => {
  const [copied, setCopied] = useState(false);

  const copyToClipboard = () => {
    navigator.clipboard.writeText(code);
    setCopied(true);
    setTimeout(() => setCopied(false), 2000);
  };

  return (
    <div className={clsx('margin-bottom--lg', styles.codeExampleContainer)}>
      <div className={styles.header}>
        <h4 className={styles.title}>{title}</h4>
        <button
          className={styles.copyButton}
          onClick={copyToClipboard}
          title="Copy to clipboard"
        >
          {copied ? '✓ Copied!' : '📋 Copy'}
        </button>
      </div>

      {description && (
        <div className={styles.description}>
          {description}
        </div>
      )}

      {safetyNotes && (
        <div className={styles.safetyNotes}>
          <strong>⚠️ Safety Notes:</strong> {safetyNotes}
        </div>
      )}

      {environment && (
        <div className={styles.environment}>
          <strong>Environment:</strong> {environment}
        </div>
      )}

      {dependencies && dependencies.length > 0 && (
        <div className={styles.dependencies}>
          <strong>Dependencies:</strong> {dependencies.join(', ')}
        </div>
      )}

      <div className={styles.codeContainer}>
        <pre className={styles.codeBlock}>
          <code className={`language-${language}`}>
            {code}
          </code>
        </pre>
      </div>
    </div>
  );
};

export default CodeExample;