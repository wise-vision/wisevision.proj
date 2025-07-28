#!/bin/bash
# filepath: /home/adam/repos/wisevision.proj/release_automation.sh

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Configuration
REPOS_FILE="project.repos"
DEFAULT_DEV_BRANCH="dev"
DEFAULT_MAIN_BRANCH="main"

# Helper functions
log_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

log_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

log_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

show_help() {
    cat << EOF
Release Automation Script for WiseVision Project

Usage: $0 [COMMAND] [OPTIONS]

Commands:
  create-stabilization    Create stabilization branches for all repos
  merge-to-main          Merge stabilization branches to main and tag
  create-hotfix          Create hotfix branches from main
  merge-hotfix           Merge hotfix branches back to main and dev
  sync-fixes             Sync fixes from stabilization back to dev
  status                 Show current branch status for all repos

Options:
  -v, --version VERSION  Release version (required for most commands)
  -f, --force           Force operations (skip confirmations)
  -d, --dry-run         Show what would be done without executing
  -h, --help            Show this help message

Examples:
  $0 create-stabilization -v 2501
  $0 merge-to-main -v 2501
  $0 create-hotfix -v 2501-1
  $0 status

EOF
}

# Parse repository paths from project.repos
get_repo_paths() {
    grep -A 3 "^  src/" "$REPOS_FILE" | grep -E "^  src/" | sed 's/://' | sort
}

# Check if a branch exists in a repository
branch_exists() {
    local repo_path=$1
    local branch_name=$2
    
    cd "$repo_path"
    git rev-parse --verify "$branch_name" >/dev/null 2>&1
    local result=$?
    cd - >/dev/null
    return $result
}

# Execute git command in a repository
exec_in_repo() {
    local repo_path=$1
    shift
    local git_cmd="$@"
    
    log_info "Executing in $repo_path: git $git_cmd"
    
    if [[ "$DRY_RUN" == "true" ]]; then
        log_warning "DRY RUN: Would execute: git $git_cmd"
        return 0
    fi
    
    cd "$repo_path"
    if git $git_cmd; then
        log_success "Success in $repo_path"
        cd - >/dev/null
        return 0
    else
        log_error "Failed in $repo_path"
        cd - >/dev/null
        return 1
    fi
}

# Create stabilization branches
create_stabilization() {
    local version=$1
    local stabilization_branch="stabilization/$version"
    
    log_info "Creating stabilization branches for version $version"
    
    local repo_paths=($(get_repo_paths))
    local failed_repos=()
    
    for repo_path in "${repo_paths[@]}"; do
        if [[ ! -d "$repo_path" ]]; then
            log_warning "Repository $repo_path not found, skipping"
            continue
        fi
        
        log_info "Processing repository: $repo_path"
        
        # Fetch latest changes
        if ! exec_in_repo "$repo_path" "fetch --all"; then
            failed_repos+=("$repo_path")
            continue
        fi
        
        # Check if dev branch exists, fallback to main
        local source_branch="$DEFAULT_DEV_BRANCH"
        if ! branch_exists "$repo_path" "origin/$DEFAULT_DEV_BRANCH"; then
            log_warning "Dev branch not found in $repo_path, using main"
            source_branch="$DEFAULT_MAIN_BRANCH"
        fi
        
        # Checkout and update source branch
        if ! exec_in_repo "$repo_path" "checkout $source_branch" || \
           ! exec_in_repo "$repo_path" "pull origin $source_branch"; then
            failed_repos+=("$repo_path")
            continue
        fi
        
        # Create stabilization branch
        if branch_exists "$repo_path" "$stabilization_branch"; then
            log_warning "Stabilization branch already exists in $repo_path"
            if [[ "$FORCE" != "true" ]]; then
                read -p "Overwrite existing branch? (y/N): " -n 1 -r
                echo
                if [[ ! $REPLY =~ ^[Yy]$ ]]; then
                    continue
                fi
            fi
            exec_in_repo "$repo_path" "branch -D $stabilization_branch" || true
        fi
        
        if ! exec_in_repo "$repo_path" "checkout -b $stabilization_branch" || \
           ! exec_in_repo "$repo_path" "push -u origin $stabilization_branch"; then
            failed_repos+=("$repo_path")
        fi
    done
    
    if [[ ${#failed_repos[@]} -gt 0 ]]; then
        log_error "Failed to create stabilization branches in: ${failed_repos[*]}"
        return 1
    fi
    
    log_success "Stabilization branches created successfully for version $version"
}

# Merge stabilization to main and tag
merge_to_main() {
    local version=$1
    local stabilization_branch="stabilization/$version"
    
    log_info "Merging stabilization branches to main and tagging version $version"
    
    local repo_paths=($(get_repo_paths))
    local failed_repos=()
    
    for repo_path in "${repo_paths[@]}"; do
        if [[ ! -d "$repo_path" ]]; then
            log_warning "Repository $repo_path not found, skipping"
            continue
        fi
        
        log_info "Processing repository: $repo_path"
        
        # Fetch latest changes
        if ! exec_in_repo "$repo_path" "fetch --all"; then
            failed_repos+=("$repo_path")
            continue
        fi
        
        # Check if stabilization branch exists
        if ! branch_exists "$repo_path" "origin/$stabilization_branch"; then
            log_warning "Stabilization branch not found in $repo_path, skipping"
            continue
        fi
        
        # Checkout main and update
        if ! exec_in_repo "$repo_path" "checkout $DEFAULT_MAIN_BRANCH" || \
           ! exec_in_repo "$repo_path" "pull origin $DEFAULT_MAIN_BRANCH"; then
            failed_repos+=("$repo_path")
            continue
        fi
        
        # Merge stabilization branch
        if ! exec_in_repo "$repo_path" "merge origin/$stabilization_branch"; then
            failed_repos+=("$repo_path")
            continue
        fi
        
        # Tag the release
        if ! exec_in_repo "$repo_path" "tag $version" || \
           ! exec_in_repo "$repo_path" "push origin $DEFAULT_MAIN_BRANCH" || \
           ! exec_in_repo "$repo_path" "push origin $version"; then
            failed_repos+=("$repo_path")
        fi
    done
    
    if [[ ${#failed_repos[@]} -gt 0 ]]; then
        log_error "Failed to merge to main in: ${failed_repos[*]}"
        return 1
    fi
    
    log_success "Successfully merged to main and tagged version $version"
}

# Sync fixes from stabilization back to dev
sync_fixes() {
    local version=$1
    local stabilization_branch="stabilization/$version"
    
    log_info "Syncing fixes from stabilization/$version back to dev"
    
    local repo_paths=($(get_repo_paths))
    local failed_repos=()
    
    for repo_path in "${repo_paths[@]}"; do
        if [[ ! -d "$repo_path" ]]; then
            log_warning "Repository $repo_path not found, skipping"
            continue
        fi
        
        # Check if dev branch exists
        if ! branch_exists "$repo_path" "origin/$DEFAULT_DEV_BRANCH"; then
            log_warning "Dev branch not found in $repo_path, skipping"
            continue
        fi
        
        # Check if stabilization branch exists
        if ! branch_exists "$repo_path" "origin/$stabilization_branch"; then
            log_warning "Stabilization branch not found in $repo_path, skipping"
            continue
        fi
        
        log_info "Processing repository: $repo_path"
        
        # Fetch and checkout dev
        if ! exec_in_repo "$repo_path" "fetch --all" || \
           ! exec_in_repo "$repo_path" "checkout $DEFAULT_DEV_BRANCH" || \
           ! exec_in_repo "$repo_path" "pull origin $DEFAULT_DEV_BRANCH"; then
            failed_repos+=("$repo_path")
            continue
        fi
        
        # Merge stabilization changes
        if ! exec_in_repo "$repo_path" "merge origin/$stabilization_branch"; then
            failed_repos+=("$repo_path")
            continue
        fi
        
        # Push updated dev branch
        if ! exec_in_repo "$repo_path" "push origin $DEFAULT_DEV_BRANCH"; then
            failed_repos+=("$repo_path")
        fi
    done
    
    if [[ ${#failed_repos[@]} -gt 0 ]]; then
        log_error "Failed to sync fixes in: ${failed_repos[*]}"
        return 1
    fi
    
    log_success "Successfully synced fixes from stabilization/$version to dev"
}

# Create hotfix branches from main for point releases
create_hotfix() {
    local version=$1
    local hotfix_branch="hotfix/$version"
    
    log_info "Creating hotfix branches for version $version"
    
    local repo_paths=($(get_repo_paths))
    local failed_repos=()
    
    for repo_path in "${repo_paths[@]}"; do
        if [[ ! -d "$repo_path" ]]; then
            log_warning "Repository $repo_path not found, skipping"
            continue
        fi
        
        log_info "Processing repository: $repo_path"
        
        # Fetch latest changes
        if ! exec_in_repo "$repo_path" "fetch --all"; then
            failed_repos+=("$repo_path")
            continue
        fi
        
        # Checkout main and update
        if ! exec_in_repo "$repo_path" "checkout $DEFAULT_MAIN_BRANCH" || \
           ! exec_in_repo "$repo_path" "pull origin $DEFAULT_MAIN_BRANCH"; then
            failed_repos+=("$repo_path")
            continue
        fi
        
        # Create hotfix branch
        if branch_exists "$repo_path" "$hotfix_branch"; then
            log_warning "Hotfix branch already exists in $repo_path"
            if [[ "$FORCE" != "true" ]]; then
                read -p "Overwrite existing branch? (y/N): " -n 1 -r
                echo
                if [[ ! $REPLY =~ ^[Yy]$ ]]; then
                    continue
                fi
            fi
            exec_in_repo "$repo_path" "branch -D $hotfix_branch" || true
        fi
        
        if ! exec_in_repo "$repo_path" "checkout -b $hotfix_branch" || \
           ! exec_in_repo "$repo_path" "push -u origin $hotfix_branch"; then
            failed_repos+=("$repo_path")
        fi
    done
    
    if [[ ${#failed_repos[@]} -gt 0 ]]; then
        log_error "Failed to create hotfix branches in: ${failed_repos[*]}"
        return 1
    fi
    
    log_success "Hotfix branches created successfully for version $version"
}

# Merge hotfix branches back to main and dev
merge_hotfix() {
    local version=$1
    local hotfix_branch="hotfix/$version"
    
    log_info "Merging hotfix branches to main and dev for version $version"
    
    local repo_paths=($(get_repo_paths))
    local failed_repos=()
    
    for repo_path in "${repo_paths[@]}"; do
        if [[ ! -d "$repo_path" ]]; then
            log_warning "Repository $repo_path not found, skipping"
            continue
        fi
        
        log_info "Processing repository: $repo_path"
        
        # Fetch latest changes
        if ! exec_in_repo "$repo_path" "fetch --all"; then
            failed_repos+=("$repo_path")
            continue
        fi
        
        # Check if hotfix branch exists
        if ! branch_exists "$repo_path" "origin/$hotfix_branch"; then
            log_warning "Hotfix branch not found in $repo_path, skipping"
            continue
        fi
        
        # Merge to main first
        if ! exec_in_repo "$repo_path" "checkout $DEFAULT_MAIN_BRANCH" || \
           ! exec_in_repo "$repo_path" "pull origin $DEFAULT_MAIN_BRANCH" || \
           ! exec_in_repo "$repo_path" "merge origin/$hotfix_branch" || \
           ! exec_in_repo "$repo_path" "push origin $DEFAULT_MAIN_BRANCH"; then
            failed_repos+=("$repo_path")
            continue
        fi
        
        # Cherry-pick to dev if dev branch exists
        if branch_exists "$repo_path" "origin/$DEFAULT_DEV_BRANCH"; then
            if ! exec_in_repo "$repo_path" "checkout $DEFAULT_DEV_BRANCH" || \
               ! exec_in_repo "$repo_path" "pull origin $DEFAULT_DEV_BRANCH" || \
               ! exec_in_repo "$repo_path" "merge origin/$hotfix_branch" || \
               ! exec_in_repo "$repo_path" "push origin $DEFAULT_DEV_BRANCH"; then
                log_warning "Failed to merge hotfix to dev in $repo_path, but main merge succeeded"
            fi
        else
            log_warning "Dev branch not found in $repo_path, skipping dev merge"
        fi
    done
    
    if [[ ${#failed_repos[@]} -gt 0 ]]; then
        log_error "Failed to merge hotfix in: ${failed_repos[*]}"
        return 1
    fi
    
    log_success "Successfully merged hotfix $version to main and dev"
}

# Show status of all repositories
show_status() {
    log_info "Showing status for all repositories"
    
    local repo_paths=($(get_repo_paths))
    
    printf "%-40s %-20s %-50s\n" "Repository" "Current Branch" "Status"
    printf "%-40s %-20s %-50s\n" "$(printf '=%.0s' {1..40})" "$(printf '=%.0s' {1..20})" "$(printf '=%.0s' {1..50})"
    
    for repo_path in "${repo_paths[@]}"; do
        if [[ ! -d "$repo_path" ]]; then
            printf "%-40s %-20s %-50s\n" "$repo_path" "N/A" "Repository not found"
            continue
        fi
        
        cd "$repo_path"
        local current_branch=$(git rev-parse --abbrev-ref HEAD 2>/dev/null || echo "N/A")
        local status=$(git status --porcelain 2>/dev/null | wc -l)
        local status_text="Clean"
        
        if [[ $status -gt 0 ]]; then
            status_text="$status uncommitted changes"
        fi
        
        printf "%-40s %-20s %-50s\n" "$repo_path" "$current_branch" "$status_text"
        cd - >/dev/null
    done
}

# Main execution
main() {
    local command=""
    local version=""
    local force=false
    local dry_run=false
    
    # Parse arguments
    while [[ $# -gt 0 ]]; do
        case $1 in
            create-stabilization|merge-to-main|create-hotfix|merge-hotfix|sync-fixes|status)
                command="$1"
                shift
                ;;
            -v|--version)
                version="$2"
                shift 2
                ;;
            -f|--force)
                force=true
                shift
                ;;
            -d|--dry-run)
                dry_run=true
                shift
                ;;
            -h|--help)
                show_help
                exit 0
                ;;
            *)
                log_error "Unknown option: $1"
                show_help
                exit 1
                ;;
        esac
    done
    
    # Set global variables
    FORCE="$force"
    DRY_RUN="$dry_run"
    
    # Validate inputs
    if [[ -z "$command" ]]; then
        log_error "No command specified"
        show_help
        exit 1
    fi
    
    if [[ "$command" != "status" && -z "$version" ]]; then
        log_error "Version is required for command: $command"
        exit 1
    fi
    
    if [[ ! -f "$REPOS_FILE" ]]; then
        log_error "Repository configuration file not found: $REPOS_FILE"
        exit 1
    fi
    
    # Execute command
    case $command in
        create-stabilization)
            create_stabilization "$version"
            ;;
        merge-to-main)
            merge_to_main "$version"
            ;;
        sync-fixes)
            sync_fixes "$version"
            ;;
        create-hotfix)
            create_hotfix "$version"
            ;;
        merge-hotfix)
            merge_hotfix "$version"
            ;;
        status)
            show_status
            ;;
        *)
            log_error "Command not implemented yet: $command"
            exit 1
            ;;
    esac
}

# Check if script is being sourced or executed
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    main "$@"
fi