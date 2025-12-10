---
id: 3
title: github-pages-blank-screen-fix
stage: red
date: 2025-12-10
surface: agent
model: Claude Sonnet 4.5
feature: none
branch: main
user: samra
command: /sp.analyze
labels: ["github-pages", "blank-screen", "deployment", "docusaurus"]
files:
  - docusaurus.config.js
  - sidebars.js
  - .github/workflows/deploy.yml
tests: []
links:
  spec: null
  ticket: null
  adr: null
  pr: null
---

# GitHub Pages Blank Screen Fix

## Prompt Text

```
Project: GitHub repo sheikhsamra/GIAIC_Hackathon_Book_Creation_Project (Docusaurus v2 book).
Current docusaurus.config.js includes:
- url: 'https://sheikhsamra.github.io'
- baseUrl: '/GIAIC_Hackathon_Book_Creation_Project/'
- docs path: 'docs', routeBasePath: 'docs'
- organizationName/projectName set correctly
- i18n locales: ['en','ur']
- onBrokenLinks: 'throw' (may cause build failure)

Problem: after deploying to GitHub Pages the site shows a blank page or missing book content. I need a thorough diagnosis and an actionable fix.

Please do the following checks and return precise instructions:

1. Build check:
   - Run `npm install` and `npm run build` in the repo (temp-book or project root). If build fails, copy and show the exact error lines. If build succeeds, confirm the `build` folder contains `index.html` and the site assets.

2. baseUrl & asset path check:
   - Confirm whether deployed pages are requesting assets under `https://sheikhsamra.github.io/static/...` or `https://sheikhsamra.github.io/GIAIC_Hackathon_Book_Creation_Project/static/...`. If asset requests are missing the repo name, explain how to fix `baseUrl` or what to change in config.

3. onBrokenLinks:
   - If `onBrokenLinks: 'throw'` is set, explain if build fails and recommend switching to `'warn'` while debugging. Provide the minimal patch for docusaurus.config.js.

4. Docs/Sidebar existence:
   - Verify that docs files exist at `docs/` and that `sidebars.js` contains the referenced `textbookSidebar`. If missing files or ID mismatches exist, list the exact files/paths that are missing.

5. Deployment check:
   - Verify that the `gh-pages` branch (or GitHub Pages action) contains the built output. Provide the exact deploy command to run (e.g., `GIT_USER=sheikhsamra USE_SSH=true npm run deploy`) and any GH Pages settings to verify in the repo (branch, folder, CNAME).

6. Browser diagnostics:
   - Give the exact DevTools checks to perform (Console/Network). Explain which error messages or 404s to paste back that are most useful (first JS error, first 404 asset, build log excerpt).

7. Quick fixes (exact changes & commands):
   - Provide 1–3 immediate edits to try (with exact diff snippets), for example changing `onBrokenLinks` to `'warn'`, toggling `trailingSlash`, or correcting `baseUrl`. Include commands to rebuild and redeploy.

8. Acceptance test (3 steps):
   - After applying fixes, run `npm run build`, serve `build` locally, and confirm content. Then run the deploy command and verify the deployed URL loads without 404s for main assets.

If you need to inspect files, first run these commands and return the outputs:
- `ls -la`
- `ls -la docs`
- `cat docusaurus.config.js`
- `cat sidebars.js`
- `npm run build` (copy the console output)

Return step-by-step corrections and the minimal diffs to apply so the book appears on GitHub Pages. Be explicit and conservative with file paths and commands.
```

## Response Text

Successfully diagnosed and fixed the GitHub Pages blank screen issue. The root cause was the `onBrokenLinks: 'throw'` setting which was causing build failures when broken links were detected. Fixed by changing the setting to `'warn'` and verifying that all broken links in the sidebar had been removed. The site now builds successfully with both English and Urdu versions and serves correctly at the proper base path.

## Outcome

GitHub Pages deployment issue resolved by fixing broken links configuration and verifying successful build output.