# Data Model: Physical AI & Humanoid Robotics Textbook

## Core Entities

### 1. TextbookModule
- **Attributes**:
  - id: string (unique identifier, e.g., "module-1", "module-2")
  - title: string (e.g., "Introduction to Physical AI", "ROS 2 Fundamentals")
  - description: string (brief overview of the module)
  - learningObjectives: array of strings (specific learning goals)
  - order: integer (sequence in the textbook, 1-6)
  - prerequisites: array of strings (required knowledge)
  - estimatedCompletionTime: integer (in minutes)
  - difficultyLevel: enum ("beginner", "intermediate", "advanced", "robotics-professional")

### 2. Chapter
- **Attributes**:
  - id: string (unique identifier within module)
  - moduleId: string (foreign key to TextbookModule)
  - title: string (chapter title)
  - content: string (main content in Markdown)
  - summary: string (brief chapter summary)
  - learningObjectives: array of strings (chapter-specific learning goals)
  - difficultyLevel: enum ("beginner", "intermediate", "advanced", "robotics-professional")
  - estimatedCompletionTime: integer (in minutes)
  - requiresSimulation: boolean (indicates if Gazebo/Isaac Sim is needed)
  - safetyConsiderations: array of strings (safety warnings for this chapter)

### 3. CodeExample
- **Attributes**:
  - id: string (unique identifier)
  - chapterId: string (foreign key to Chapter)
  - title: string (description of the example)
  - language: string (e.g., "python", "yaml", "xml", "bash")
  - code: string (the actual code content)
  - description: string (explanation of what the code does)
  - safetyNotes: string (any safety warnings for the code)
  - environment: enum ("ros2-humble", "gazebo", "isaac-sim", "simulation-only", "hardware")
  - dependencies: array of strings (required packages or libraries)
  - executionInstructions: string (how to run the example)

### 4. LabExercise
- **Attributes**:
  - id: string (unique identifier)
  - chapterId: string (foreign key to Chapter)
  - title: string (exercise title)
  - objectives: array of strings (what the exercise teaches)
  - instructions: string (step-by-step instructions)
  - requirements: array of strings (prerequisites, hardware, software)
  - expectedOutcome: string (what the learner should achieve)
  - difficultyLevel: enum ("beginner", "intermediate", "advanced", "robotics-professional")
  - estimatedDuration: integer (in minutes)
  - safetyRequirements: array of strings (safety measures required)
  - evaluationCriteria: array of strings (how the exercise will be evaluated)

### 5. Quiz
- **Attributes**:
  - id: string (unique identifier)
  - chapterId: string (foreign key to Chapter)
  - title: string (quiz title)
  - description: string (purpose of the quiz)
  - questions: array of Question objects
  - passingScore: number (minimum score to pass, 0-100)
  - timeLimit: integer (in minutes, 0 if no limit)
  - difficultyLevel: enum ("beginner", "intermediate", "advanced", "robotics-professional")

### 6. Question
- **Attributes**:
  - id: string (unique identifier)
  - quizId: string (foreign key to Quiz)
  - text: string (the question text)
  - type: enum ("multiple-choice", "true-false", "short-answer", "essay", "code-completion", "ordering")
  - options: array of strings (for multiple choice and ordering)
  - correctAnswer: string | array (the correct answer(s))
  - explanation: string (why this is the correct answer)
  - points: integer (how many points this question is worth)
  - difficulty: enum ("easy", "medium", "hard")

### 7. PersonalizedContent
- **Attributes**:
  - id: string (unique identifier)
  - chapterId: string (foreign key to Chapter)
  - userLevel: enum ("beginner", "intermediate", "advanced", "robotics-professional")
  - content: string (personalized version of the chapter content)
  - depthLevel: enum ("overview", "detailed", "comprehensive", "expert")
  - examplesCount: integer (number of examples to include)
  - explanationStyle: enum ("simplified", "standard", "technical", "comprehensive")

### 8. Translation
- **Attributes**:
  - id: string (unique identifier)
  - chapterId: string (foreign key to Chapter)
  - languageCode: string (e.g., "ur", "en")
  - title: string (translated title)
  - content: string (translated content)
  - technicalTerms: array of strings (English terms preserved in translation)
  - translatorNotes: string (notes from translator about translation choices)
  - reviewed: boolean (has the translation been reviewed for accuracy)

### 9. Diagram
- **Attributes**:
  - id: string (unique identifier)
  - chapterId: string (foreign key to Chapter)
  - title: string (diagram title)
  - description: string (what the diagram shows)
  - type: enum ("ascii", "uml", "flowchart", "architecture", "circuit", "block-diagram", "sequence")
  - content: string (diagram content or reference)
  - altText: string (alternative text for accessibility)
  - sourceFile: string (path to source file if applicable)

### 10. UserProgress
- **Attributes**:
  - id: string (unique identifier)
  - userId: string (user identifier)
  - chapterId: string (foreign key to Chapter)
  - status: enum ("not-started", "in-progress", "completed")
  - completionPercentage: number (0-100)
  - timeSpent: integer (in minutes)
  - lastAccessed: date (when the user last accessed this chapter)
  - quizScores: array of objects (quizId and score pairs)
  - personalizedLevel: enum ("beginner", "intermediate", "advanced", "robotics-professional")

### 11. RAGKnowledgeBase
- **Attributes**:
  - id: string (unique identifier)
  - contentId: string (reference to source content)
  - text: string (the text content for RAG)
  - embeddings: array of numbers (vector embeddings for similarity search)
  - metadata: object (additional information like source, chapter, module)
  - version: string (version of the content)
  - createdAt: date (when this entry was created)
  - updatedAt: date (when this entry was last updated)

## Relationships

- TextbookModule 1 → * Chapter (one module has many chapters)
- Chapter 1 → * CodeExample (one chapter has many code examples)
- Chapter 1 → * LabExercise (one chapter has many lab exercises)
- Chapter 1 → * Quiz (one chapter has one quiz)
- Chapter 1 → * Diagram (one chapter has many diagrams)
- Quiz 1 → * Question (one quiz has many questions)
- Chapter 1 → * PersonalizedContent (one chapter has multiple personalized versions)
- Chapter 1 → * Translation (one chapter has multiple translations)
- User 1 → * UserProgress (one user has progress for many chapters)
- Chapter 1 → * UserProgress (one chapter has progress for many users)
- Content 1 → * RAGKnowledgeBase (one content piece maps to one RAG entry)