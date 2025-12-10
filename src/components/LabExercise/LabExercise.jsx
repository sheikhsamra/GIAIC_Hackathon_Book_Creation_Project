import React, { useState } from 'react';
import clsx from 'clsx';
import styles from './LabExercise.module.css';

const LabExercise = ({ labData, title = "Lab Exercise" }) => {
  const [completedSteps, setCompletedSteps] = useState({});
  const [showSafety, setShowSafety] = useState(false);

  if (!labData) {
    return null;
  }

  const toggleStep = (stepIndex) => {
    setCompletedSteps(prev => ({
      ...prev,
      [stepIndex]: !prev[stepIndex]
    }));
  };

  const allStepsCompleted = labData.steps &&
    Object.keys(completedSteps).length === labData.steps.length &&
    labData.steps.every((_, index) => completedSteps[index]);

  return (
    <div className={clsx('margin-bottom--lg', styles.labContainer)}>
      <div className={styles.header}>
        <h3 className={styles.labTitle}>{title}</h3>
        {labData.safetyRequirements && labData.safetyRequirements.length > 0 && (
          <button
            className={styles.safetyButton}
            onClick={() => setShowSafety(!showSafety)}
          >
            ⚠️ Safety
          </button>
        )}
      </div>

      {labData.safetyRequirements && labData.safetyRequirements.length > 0 && showSafety && (
        <div className={styles.safetyContainer}>
          <h4 className={styles.safetyTitle}>⚠️ Safety Requirements</h4>
          <ul className={styles.safetyList}>
            {labData.safetyRequirements.map((req, index) => (
              <li key={index} className={styles.safetyItem}>{req}</li>
            ))}
          </ul>
        </div>
      )}

      {labData.objectives && labData.objectives.length > 0 && (
        <div className={styles.objectivesContainer}>
          <h4 className={styles.objectivesTitle}>Objectives</h4>
          <ul className={styles.objectivesList}>
            {labData.objectives.map((objective, index) => (
              <li key={index} className={styles.objectiveItem}>{objective}</li>
            ))}
          </ul>
        </div>
      )}

      {labData.requirements && labData.requirements.length > 0 && (
        <div className={styles.requirementsContainer}>
          <h4 className={styles.requirementsTitle}>Requirements</h4>
          <ul className={styles.requirementsList}>
            {labData.requirements.map((req, index) => (
              <li key={index} className={styles.requirementItem}>{req}</li>
            ))}
          </ul>
        </div>
      )}

      {labData.steps && labData.steps.length > 0 && (
        <div className={styles.stepsContainer}>
          <h4 className={styles.stepsTitle}>Steps</h4>
          <ol className={styles.stepsList}>
            {labData.steps.map((step, index) => (
              <li key={index} className={clsx(styles.stepItem, completedSteps[index] && styles.stepCompleted)}>
                <label className={styles.stepLabel}>
                  <input
                    type="checkbox"
                    checked={!!completedSteps[index]}
                    onChange={() => toggleStep(index)}
                    className={styles.stepCheckbox}
                  />
                  <span className={styles.stepText}>{step}</span>
                </label>
              </li>
            ))}
          </ol>
        </div>
      )}

      {labData.expectedOutcome && (
        <div className={styles.outcomeContainer}>
          <h4 className={styles.outcomeTitle}>Expected Outcome</h4>
          <p className={styles.outcomeText}>{labData.expectedOutcome}</p>
        </div>
      )}

      {allStepsCompleted && (
        <div className={styles.completionMessage}>
          <div className={styles.completionIcon}>✓</div>
          <p className={styles.completionText}>Congratulations! You've completed this lab exercise.</p>
        </div>
      )}
    </div>
  );
};

export default LabExercise;