# Issue tracker: Local Markdown

Issues and specs for this repo live as markdown files in `.scratch/`.

## Conventions

- One feature per directory: `.scratch/<feature-slug>/`
- The spec is `.scratch/<feature-slug>/spec.md`
- Implementation issues are one file per ticket at `.scratch/<feature-slug>/issues/<NN>-<slug>.md`, numbered from `01`, never a single combined tickets file
- Triage state is recorded as a `Status:` line near the top of each issue file (role strings come from `triage-labels.md` when it exists)
- Triaged issues also carry a `Category:` line (`bug` or `enhancement`)
- Incoming reports that belong to no feature yet go in `.scratch/inbox/issues/<NN>-<slug>.md`
- Comments and conversation history append to the bottom of the file under a `## Comments` heading
- **Blocking**: a `Blocked by: NN, NN` line near the top. An issue is unblocked when every issue it lists is closed.
- **Close**: add a `Closed: <date>` line under the `Status:` line, and append the reason under `## Comments`

## When a skill says "publish to the issue tracker"

Create a new file under `.scratch/<feature-slug>/` (creating the directory if needed).

## When a skill says "fetch the issue"

Read the file at the referenced path. The user will normally pass the path or the issue number directly.

## Wayfinding operations

Used by `/wayfinder`. The **map** is a file with one **child** file per ticket. Children live in `decisions/`, apart from the implementation issues in `issues/`, because their `Status:` line means something else.

- **Map**: `.scratch/<effort>/map.md` (the Destination / Notes / Decisions so far / Added so far / Not yet specified / Out of scope body).
- **Child ticket**: `.scratch/<effort>/decisions/NN-<slug>.md`, numbered from `01`, with the question in the body. A `Type:` line records the ticket type (`research`/`prototype`/`grilling`/`task`); a `Status:` line records `claimed`/`resolved`.
- **Blocking**: a `Blocked by: NN, NN` line near the top. A ticket is unblocked when every file it lists is `resolved` or closed (the Close operation above).
- **Frontier**: scan `.scratch/<effort>/decisions/` for files that are open, unblocked, and unclaimed; first by number wins.
- **Claim**: set `Status: claimed` and save before any work.
- **Resolve**: append the answer under an `## Answer` heading, set `Status: resolved`, then append a context pointer (gist + link) to the map's Decisions-so-far in `map.md`.
