import React, { useState } from 'react';

// A simple quiz component for self-assessment
const Quiz = ({ questions, title = 'Self-Assessment Quiz' }) => {
  const [answers, setAnswers] = useState({});
  const [submitted, setSubmitted] = useState(false);

  const handleAnswerChange = (questionIndex, value) => {
    setAnswers({
      ...answers,
      [questionIndex]: value
    });
  };

  const handleSubmit = (e) => {
    e.preventDefault();
    setSubmitted(true);
  };

  const calculateScore = () => {
    if (!submitted) return 0;
    let correct = 0;
    questions.forEach((q, i) => {
      if (answers[i] === q.correctAnswer) correct++;
    });
    return Math.round((correct / questions.length) * 100);
  };

  return (
    <div className="quiz-component">
      <h3>{title}</h3>
      <form onSubmit={handleSubmit}>
        {questions.map((question, index) => (
          <div key={index} className="quiz-question">
            <h4>{question.question}</h4>
            <ul className="quiz-options">
              {question.options.map((option, optIndex) => (
                <li key={optIndex}>
                  <label>
                    <input
                      type="radio"
                      name={`question-${index}`}
                      value={option.value}
                      onChange={(e) => handleAnswerChange(index, e.target.value)}
                      disabled={submitted}
                    />
                    {option.label}
                  </label>
                </li>
              ))}
            </ul>
            {submitted && answers[index] === question.correctAnswer && (
              <div className="correct-answer">✓ Correct!</div>
            )}
            {submitted && answers[index] !== question.correctAnswer && (
              <div className="incorrect-answer">✗ Incorrect. Correct answer: {question.options.find(opt => opt.value === question.correctAnswer)?.label}</div>
            )}
          </div>
        ))}
        {!submitted && (
          <button type="submit" className="submit-quiz-btn">Submit Quiz</button>
        )}
      </form>
      {submitted && (
        <div className="quiz-results">
          <h4>Results: {calculateScore()}%</h4>
        </div>
      )}
    </div>
  );
};

export default Quiz;