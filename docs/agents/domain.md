# Domain Docs

How the engineering skills should consume this repo's domain documentation when exploring the codebase.

## Before exploring, read these

- `CONTEXT.md` at the repo root, if it exists.
- `docs/adr/`, reading ADRs that touch the area being worked on.

If these files don't exist, proceed silently. The `/domain-modeling` skill creates them lazily when terms or decisions actually get resolved.

## File structure

Single-context repo:

```text
/
|-- CONTEXT.md
|-- docs/adr/
`-- src/
```

## Use the glossary's vocabulary

When output names a domain concept, use the term as defined in `CONTEXT.md`.

If the concept isn't in the glossary yet, either reconsider the language or note the gap for `/domain-modeling`.

## Flag ADR conflicts

If output contradicts an existing ADR, surface it explicitly rather than silently overriding.
