# Docker Usage in Jenkins Pipelines

This document explains how Jenkins pipelines can run Docker commands (`docker build`, `docker run`, etc.) on Construct.

---

## How it works

The bootstrap script (`jenkins_bootstrap.sh`) starts Jenkins with Docker group access using `sg docker ...`, which allows the Jenkins process to access `/var/run/docker.sock`.

This enables Docker commands in pipeline steps without requiring `sudo`.

---

## Verify Docker access

Run these commands on the Construct machine:

```bash
docker version
ls -l /var/run/docker.sock
```

---

## Pipeline example

```groovy
stage('Build') {
    steps {
        sh 'docker build -t myimage:ci .'
    }
}

stage('Test') {
    steps {
        sh 'docker run --rm myimage:ci'
    }
}
```

---

## Security note

> ⚠️ **Warning**: Access to the Docker socket is effectively root-equivalent on the host.

This is acceptable for a single-user Construct VM, but **not recommended for shared or production environments**.
