# Jenkins job setup (GitHub App + Multibranch + Webhooks via smee)

This guide assumes:

- Jenkins is running (via `scripts/jenkins_bootstrap.sh`)
- You are using a GitHub App for Jenkins authentication
- On Construct, GitHub cannot reach Jenkins directly, so we forward webhooks using smee.io

---

## 1) Install required plugins

Manage Jenkins -> Plugins -> Available plugins

Install (minimum):

- workflow-aggregator (Pipeline)
- git
- github
- github-branch-source (Multibranch)
- github-checks (recommended: publishes GitHub Checks UI)
- role-strategy (optional)

Restart Jenkins if prompted.

---

## 2) Create a GitHub App for Jenkins

Create a GitHub App (GitHub -> Settings -> Developer settings -> GitHub Apps -> New GitHub App)

Recommended **Repository permissions** for Jenkins CI:

- Contents: Read-only (so Jenkins can read repo content)
- Metadata: Read-only
- Pull requests: Read-only (for PR discovery)
- Checks: Read & write (needed for GitHub Checks publishing)

Subscribe to events (typical for CI):

- Push
- Pull request
- (optional) Check suite / Check run

Generate a **private key** for the app (you download a `.pem`).

Install the app on the repos you want Jenkins to build (Repo access: select the specific repos or “all repos”).

References:

- GitHub Branch Source plugin: GitHub App auth guide
- GitHub Checks plugin: requires GitHub App with Checks permission

## 3) Add the GitHub App credential to Jenkins

Jenkins -> Manage Jenkins -> Credentials -> (global) -> Add Credentials

Kind: **GitHub App**

- App ID: (from your GitHub App page)
- Private Key: upload/paste the `.pem` you downloaded

Note: some setups require converting the key format (PKCS8). If Jenkins rejects the key, see:

- `jenkins-infra/jenkins/troubleshooting.md` (or add a section below)

Reference:

- GitHub Branch Source plugin GitHub App doc

## 4) Create the Multibranch Pipeline job

New Item -> Multibranch Pipeline

Branch Sources -> Add source -> GitHub

- Credentials: select your GitHub App credential
- Owner: your org/user
- Repository: your repo name
- Behaviors:
  - Add → **Discover pull requests from origin** → Strategy: **Merging the pull request with the current target branch revision**
    (This tests the PR as if it were already merged into the target branch)
  - Add → **Status Check Properties** → check **Skip GitHub Branch Source notifications**
    (This prevents 403 errors from legacy commit status API and uses Checks only)

Build Configuration:

- Mode: by Jenkinsfile
- Script Path: Jenkinsfile

Save.

Jenkins will scan branches and create child jobs automatically.

## 5) Webhook delivery on Construct (via smee.io)

Since Construct environments are not directly reachable from the public internet, GitHub cannot deliver webhooks to Jenkins directly. We use [smee.io](https://smee.io) as a webhook proxy to bridge this gap.

### How it works

```
GitHub App → smee.io (public) → smee client (local) → Jenkins
```

1. The webhook URL in your **GitHub App settings** points to your smee.io channel (e.g., `https://smee.io/your-channel`)
2. When events occur (push, PR, etc.), GitHub sends them to smee.io
3. The smee client running locally receives these events and forwards them to Jenkins at `/github-webhook/`

This configuration is set once in the GitHub App — no per-repository webhook setup required.

### Starting the smee forwarder

If `SMEE_URL` is defined in your environment (e.g., in `.bashrc`), `jenkins_bootstrap.sh` automatically starts the smee forwarder.

To start manually:

```bash
cd jenkins-infra/scripts
SMEE_URL="https://smee.io/yourchannel" SLOT_PREFIX="$SLOT_PREFIX" bash start_smee.sh
```

---

## 6) Verify the setup

Push a commit to trigger a build. If everything is configured correctly:

1. **smee.io** forwards the webhook (check terminal for POST logs)
2. **Jenkins** triggers the pipeline automatically
3. **GitHub** displays a check run on the commit (via GitHub Checks plugin)

---

## Notes

**We use GitHub Checks only** — This setup uses the modern GitHub Checks UI instead of legacy commit statuses. The "Skip GitHub Branch Source notifications" behavior (configured in step 4) prevents 403 errors from the commit status API.
