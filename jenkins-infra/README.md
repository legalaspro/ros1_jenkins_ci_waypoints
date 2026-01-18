# Jenkins Infrastructure

Scripts and documentation for bootstrapping Jenkins and connecting it to GitHub via webhooks.

> **Note**: Originally developed for [Construct](https://www.theconstruct.ai/) environments, but easily adapted to any Jenkins setup. The bootstrap script includes workarounds for Ubuntu 20.04 (Focal) — on newer systems (22.04+), these can be simplified or removed.

---

## Quick Access (Construct)

To launch Jenkins and view all CI jobs:

```bash
cd jenkins-infra/scripts
bash jenkins_bootstrap.sh
cat ~/jenkins_pid_url.txt
```

Open the URL and login with `test` / `test`. All jobs, build history, and logs are visible.

---

## Quick Start

### 1. Run the bootstrap script

```bash
cd jenkins-infra/scripts
bash jenkins_bootstrap.sh
```

This will:

- Install Docker and Java 21 (if missing)
- Download and start Jenkins
- Start the smee forwarder automatically (if `SMEE_URL` is set in your environment)

The script prints the Jenkins URL and writes a state file with PID + URLs.

### 2. Unlock Jenkins

```bash
cat ~/webpage_ws/jenkins/secrets/initialAdminPassword
```

### 3. Install plugins and configure the job

Follow the setup guide:

- `jenkins/plugins.txt` — plugin list with versions
- `jenkins/job-setup.md` — complete setup steps (plugins, GitHub App, Multibranch Pipeline)

### 4. (Optional) Start smee manually

If `SMEE_URL` was not set during bootstrap, you can start the forwarder later:

```bash
cd jenkins-infra/scripts
SMEE_URL="https://smee.io/yourchannel" bash start_smee.sh
```

---

## Files

| File                           | Description                                                    |
| ------------------------------ | -------------------------------------------------------------- |
| `scripts/jenkins_bootstrap.sh` | Installs dependencies, starts Jenkins (and smee if configured) |
| `scripts/start_smee.sh`        | Start smee forwarder manually                                  |
| `scripts/stop_smee.sh`         | Stop smee forwarder                                            |
| `scripts/install_plugins.sh`   | Automated plugin installation via Plugin Installation Manager  |
| `jenkins/plugins.txt`          | Plugin list with pinned versions                               |
| `jenkins/job-setup.md`         | Job + GitHub App + webhook setup guide                         |
| `jenkins/docker.md`            | Docker usage in Jenkins pipelines                              |

---

## Notes

- **Test user**: A read-only user is available for viewing jobs: username `test`, password `test`.

- **Smee forwarder**: Useful when Jenkins runs on a dynamic or private machine that GitHub cannot reach directly (e.g., cloud VMs with changing IPs, machines behind firewalls). [smee.io](https://smee.io) provides a stable public URL that forwards webhook events to your local Jenkins. Set `SMEE_URL` in `.bashrc` for automatic startup.

- **Reverse proxy**: If Jenkins runs behind a reverse proxy, use the public URL shown by the bootstrap script.
