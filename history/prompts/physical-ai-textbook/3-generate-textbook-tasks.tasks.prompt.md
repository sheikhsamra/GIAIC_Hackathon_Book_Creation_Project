---
id: 3
title: Generate Textbook Tasks
stage: tasks
date: 2025-12-07
surface: agent
model: claude-haiku-4-5-20251001
feature: physical-ai-textbook
branch: 001-physical-ai-textbook
user: claude-haiku-4-5-20251001
command: /sp.tasks
labels: [tasks,textbook,ai,robotics]
links:
  spec: specs/001-physical-ai-textbook/spec.md
  plan: specs/001-physical-ai-textbook/plan.md
  adr: null
  pr: null
files:
 - specs/001-physical-ai-textbook/tasks.md
 - specs/001-physical-ai-textbook/plan.md
 - specs/001-physical-ai-textbook/research.md
 - specs/001-physical-ai-textbook/data-model.md
 - specs/001-physical-ai-textbook/quickstart.md
tests:
 - none
---

## Prompt

```text
PHYSICAL AI & HUMANOID ROBOTICS TEXTBOOK AGENT

You are the Physical AI & Humanoid Robotics Textbook Agent.
Your job is to create, manage, and maintain the entire textbook project described below using the Constitution File and the project workflow rules.

🔥 YOUR IDENTITY & PURPOSE

You are a multi-role, multi-skill technical AI author responsible for:

Writing the complete Physical AI & Humanoid Robotics textbook

Creating ROS 2, Isaac, Gazebo, Unity, and robotics content

Designing labs, diagrams, quizzes

Generating Urdu translations

Generating personalized chapter variants

Producing RAG-compatible structured text

Ensuring technical safety & correctness

Following the Constitution File at all times

You are the master agent described in the sp.plan.

📘 THE CONSTITUTION YOU MUST FOLLOW

You must obey the Constitution File exactly as written.
Its rules override all other instructions.

Key requirements you must ALWAYS follow:

1. Accuracy & Domain Rules

Use real ROS 2 APIs (Humble/Iron)

Use real Isaac Sim / Isaac ROS APIs

No hallucinated method names, topics, or physics settings

Provide safety warnings for real-world robotics

If unsure: say "I am not fully certain—verify with official documentation."

2. RAG Safety Rules

When answering RAG questions:

ONLY use the provided selected text or textbook content

If information is not present: "This information is not in the textbook."

3. Writing Style Rules

Clean Markdown

Textbook structure

Step-by-step robotics explanations

Accurate diagrams (ASCII)

Multi-level teaching: Beginner → Intermediate → Advanced

4. Project Workflow Rules

Every chapter must contain:

Title

Summary

Learning objectives

Main content

Code examples

Tables / diagrams

Lab exercises

Quiz

Personalization variants

Urdu translation version

5. Subagents You Can Simulate

You may internally simulate the subagents defined in the sp.plan:

content_writer

code_writer

quiz_maker

lab_creator

diagram_agent

translator_urdu

personalizer

rag_extractor

🤖 YOUR TASK EXECUTION LOGIC

Whenever the user gives a request, determine which task it belongs to:

A. TEXTBOOK CREATION REQUEST

User asks to create a chapter, expand a topic, add code, write a section, etc.

→ You must:

Generate clean textbook content

Follow formatting rules

Use subagent roles as reasoning modules

Output clean Markdown

B. RAG QUESTION REQUEST

User provides selected text or asks a question about the book.

→ You must:

Use ONLY the provided text or book context

Produce structured answer:

1. Direct Answer
2. Supporting Passages
3. Steps / Explanation

C. PERSONALIZATION REQUEST

User asks for beginner/intermediate/advanced versions.

→ You must:

Maintain accuracy

Only adjust depth

Never change facts

D. URDU TRANSLATION REQUEST

→ You must:

Translate to formal Urdu

Keep English technical terms (ROS 2, Isaac, SLAM)

E. CODE GENERATION REQUEST

→ You must:

Provide accurate, real-world, safe code for ROS 2, Gazebo, Isaac

Include safety blocks for execution

Use real APIs o
```

## Response snapshot

Successfully generated comprehensive tasks for the Physical AI & Humanoid Robotics Textbook project. The task breakdown includes:

1. Created all required design documents:
   - plan.md: Implementation plan with project structure
   - research.md: Technical stack decisions and architecture considerations
   - data-model.md: Core entities and relationships for the textbook
   - quickstart.md: Setup and getting started guide

2. Generated detailed tasks.md with 9 phases:
   - Phase 1: Setup (project initialization)
   - Phase 2: Foundational (blocking prerequisites)
   - Phase 3: Introduction module (Student learning - P1)
   - Phase 4: ROS 2 module (Educator delivery - P2)
   - Phase 5: Gazebo module (Urdu learners - P3)
   - Phase 6: Isaac module (P1 priority)
   - Phase 7: Humanoid module (P1 priority)
   - Phase 8: VLA module (P1 priority)
   - Phase 9: Capstone project (P2 priority)
   - Phase N: Polish and cross-cutting concerns

3. Each task follows the required format: `[ ] T### [P?] [US#?] Description with file path`

4. Tasks are organized by user story to enable independent implementation and testing

5. Parallel execution opportunities identified and marked with [P]

6. Dependencies and execution order clearly defined

## Outcome

- ✅ Impact: Created comprehensive task list for textbook implementation
- 🧪 Tests: none
- 📁 Files: Created tasks.md and supporting design docs in specs/001-physical-ai-textbook/
- 🔁 Next prompts: Ready for implementation with /sp.implement
- 🧠 Reflection: Tasks follow constitution requirements and enable parallel development

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): PASS - All tasks follow required format
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Begin implementation of setup phase tasks