# Plugin Installation (Manual)

## Where

**Manage Jenkins → Plugins → Available plugins**

## What to Install

Use `plugins.txt` as the checklist.

### Required (minimum)

| Plugin                 | Purpose                         |
| ---------------------- | ------------------------------- |
| `workflow-aggregator`  | Pipeline support                |
| `git`                  | Git SCM integration             |
| `github`               | GitHub integration              |
| `github-branch-source` | Multibranch Pipeline for GitHub |
| `github-checks`        | Publish to GitHub Checks UI     |

### Optional

| Plugin                | Purpose                                  |
| --------------------- | ---------------------------------------- |
| `role-strategy`       | Role-based access control                |
| `docker-workflow`     | Jenkins "agent docker" features          |
| `credentials-binding` | Helper for secrets/env vars in pipelines |

## After Install

Restart Jenkins if prompted.

## Automated Installation

```bash
cd jenkins-infra/scripts
bash install_plugins.sh
```
