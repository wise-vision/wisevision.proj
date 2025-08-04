## Release Management

The project includes an automated release management script that handles the multi-repository workflow for creating releases, stabilization branches, and hotfixes.

### Setup

Make the script executable:
```bash
chmod +x docs/release_automation/release_automation.sh
```

### Repository File Detection

The script automatically detects repository configuration files in this order:
1. File specified with `-r/--repos-file` option
2. `project.repos` in current directory
3. `project.repos` in parent directories (up to 3 levels)
4. `image.repos` files in `docker-files/*` subdirectories

### Common Release Workflow

1. **Create stabilization branches** for all repositories:
```bash
./docs/release_automation/release_automation.sh create-stabilization -v 2508
```

2. **Check status** of all repositories:
```bash
./docs/release_automation/release_automation.sh status
```

3. **Sync fixes** from stabilization back to dev (during stabilization phase):
```bash
./docs/release_automation/release_automation.sh sync-fixes -v 2508
```

4. **Merge to main and tag** the release (final step):
```bash
./docs/release_automation/release_automation.sh merge-to-main -v 2508
```

### Working with Different Repository Files

**Use specific repos file:**
```bash
./docs/release_automation/release_automation.sh status -r docker-files/wisevision-base-image/image.repos
```

**Run from different project directory:**
```bash
./docs/release_automation/release_automation.sh create-stabilization -v 2508 -p /path/to/project
```

**Combined usage:**
```bash
./docs/release_automation/release_automation.sh merge-to-main -v 2508 \
  -r custom.repos -p /path/to/project
```

### Available Commands

- `create-stabilization -v VERSION` - Create stabilization branches from dev
- `merge-to-main -v VERSION` - Merge stabilization to main and tag release
- `sync-fixes -v VERSION` - Sync fixes from stabilization back to dev
- `status` - Show current branch status for all repositories

### Options

- `-v, --version VERSION` - Release version (required for most commands)
- `-r, --repos-file FILE` - Path to repos file (auto-detected if not specified)
- `-p, --project-path PATH` - Base path for project (default: current directory)
- `-f, --force` - Force operations (skip confirmations)
- `-d, --dry-run` - Show what would be done without executing
- `-h, --help` - Show detailed help

### Examples

**Basic usage (auto-detect repos file):**
```bash
./docs/release_automation/release_automation.sh status
```

**Use specific image.repos file:**
```bash
./docs/release_automation/release_automation.sh create-stabilization -v 2508 \
  -r docker-files/wisevision-base-image/image.repos
```

**Work with project in different location:**
```bash
./docs/release_automation/release_automation.sh status -p /home/user/other-project
```

**Dry run to see what would happen:**
```bash
./docs/release_automation/release_automation.sh create-stabilization -v 2508 --dry-run
```

**Force operations without prompts:**
```bash
./docs/release_automation/release_automation.sh merge-to-main -v 2508 --force
```