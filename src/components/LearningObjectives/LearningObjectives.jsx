import React from 'react';
import clsx from 'clsx';
import styles from './LearningObjectives.module.css';

// Define the LearningObjectives component
// This component displays learning objectives in a structured format
const LearningObjectives = ({ objectives, title = "Learning Objectives" }) => {
  if (!objectives || !Array.isArray(objectives) || objectives.length === 0) {
    return null;
  }

  return (
    <div className={clsx('margin-bottom--md', styles.container)}>
      <h3 className={styles.title}>{title}</h3>
      <ul className={styles.objectivesList}>
        {objectives.map((objective, index) => (
          <li key={index} className={styles.objectiveItem}>
            {objective}
          </li>
        ))}
      </ul>
    </div>
  );
};

export default LearningObjectives;