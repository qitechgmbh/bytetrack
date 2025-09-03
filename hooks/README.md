# Git Hooks

This directory contains Git hooks for the project.

## Installation

To install the hooks, run the following command from the repository root:

```bash
# Copy the pre-commit hook to your local .git/hooks directory
cp hooks/pre-commit .git/hooks/pre-commit
chmod +x .git/hooks/pre-commit
```

## Hooks

### pre-commit

This hook runs the `bytetrack/rustdoc.sh` script before each commit to:
- Generate up-to-date Rust documentation
- Automatically stage the generated `rustdoc.md` file if it was modified

The commit will fail if the rustdoc generation fails, ensuring that documentation is always up-to-date.
