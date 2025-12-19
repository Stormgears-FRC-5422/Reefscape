# Reefscape

- **Required JDK:** Java 17 (tested with Eclipse Temurin 17)

To ensure consistent builds, prefer configuring the JDK locally rather than committing
machine-specific paths into the repository.

Recommended options:

- Set `JAVA_HOME` dynamically on macOS (zsh):

```bash
echo 'export JAVA_HOME=$(/usr/libexec/java_home -v17)' >> ~/.zshrc
source ~/.zshrc
```

- Or set `JAVA_HOME` temporarily when running Gradle:

```bash
JAVA_HOME=$(/usr/libexec/java_home -v17) ./gradlew build
```

- Prefer Gradle toolchains so Gradle requests Java 17 automatically (add to `build.gradle`):

```groovy
java {
	toolchain {
		languageVersion = JavaLanguageVersion.of(17)
	}
}
```

The CI workflow already sets up Java 17 via `actions/setup-java`, so no local export is
needed for CI runs.
