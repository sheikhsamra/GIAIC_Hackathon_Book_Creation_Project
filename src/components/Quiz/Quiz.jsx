import React, { useState } from 'react';
import clsx from 'clsx';
import styles from './Quiz.module.css';

const Quiz = ({ quizData, title = "Knowledge Check" }) => {
  const [answers, setAnswers] = useState({});
  const [submitted, setSubmitted] = useState(false);
  const [showResults, setShowResults] = useState(false);

  if (!quizData || !Array.isArray(quizData.questions) || quizData.questions.length === 0) {
    return null;
  }

  const handleAnswerChange = (questionId, value) => {
    if (!submitted) {
      setAnswers(prev => ({
        ...prev,
        [questionId]: value
      }));
    }
  };

  const handleSubmit = (e) => {
    e.preventDefault();
    setSubmitted(true);
    setShowResults(true);
  };

  const calculateScore = () => {
    if (!showResults) return 0;

    let correct = 0;
    quizData.questions.forEach(question => {
      if (answers[question.id] === question.correctAnswer) {
        correct++;
      }
    });
    return Math.round((correct / quizData.questions.length) * 100);
  };

  const score = calculateScore();

  return (
    <div className={clsx('margin-bottom--lg', styles.quizContainer)}>
      <h3 className={styles.quizTitle}>{title}</h3>

      <form onSubmit={handleSubmit} className={styles.quizForm}>
        {quizData.questions.map((question, index) => {
          const userAnswer = answers[question.id];
          const isCorrect = userAnswer === question.correctAnswer;
          const isAnswered = userAnswer !== undefined;

          return (
            <div key={question.id} className={styles.questionContainer}>
              <h4 className={styles.questionText}>
                {index + 1}. {question.text}
              </h4>

              <div className={styles.optionsContainer}>
                {question.options.map((option, optIndex) => (
                  <label
                    key={optIndex}
                    className={clsx(
                      styles.optionLabel,
                      !submitted && 'cursor-pointer',
                      submitted && option === question.correctAnswer && styles.correctOption,
                      submitted && option === userAnswer && option !== question.correctAnswer && styles.incorrectOption
                    )}
                  >
                    <input
                      type="radio"
                      name={`question-${question.id}`}
                      value={option}
                      checked={userAnswer === option}
                      onChange={(e) => handleAnswerChange(question.id, e.target.value)}
                      disabled={submitted}
                      className={styles.optionInput}
                    />
                    <span className={styles.optionText}>{option}</span>
                  </label>
                ))}
              </div>

              {submitted && isAnswered && (
                <div className={clsx(styles.explanation, isCorrect ? styles.correctExplanation : styles.incorrectExplanation)}>
                  <strong>{isCorrect ? '✓ Correct!' : '✗ Incorrect.'}</strong> {question.explanation}
                </div>
              )}
            </div>
          );
        })}

        {!submitted && (
          <button type="submit" className={styles.submitButton}>
            Submit Answers
          </button>
        )}
      </form>

      {showResults && (
        <div className={styles.resultsContainer}>
          <h4 className={styles.resultsTitle}>Your Results</h4>
          <p className={styles.score}>Score: {score}% ({score === 100 ? 'Perfect!' : score >= 70 ? 'Good job!' : 'Keep studying!'})</p>
          <button
            className={styles.resetButton}
            onClick={() => {
              setAnswers({});
              setSubmitted(false);
              setShowResults(false);
            }}
          >
            Retake Quiz
          </button>
        </div>
      )}
    </div>
  );
};

export default Quiz;