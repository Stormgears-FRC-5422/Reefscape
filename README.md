# Reefscape

- **Required JDK:** Java 17 (tested with Eclipse Temurin 17)

To ensure consistent builds, this repository pins the JDK for Gradle in `gradle.properties`.

If you need to change the JDK used for local builds, either:

- Update the `org.gradle.java.home` path in `gradle.properties`.
- Or set `JAVA_HOME` before running Gradle, for example:

```bash
export JAVA_HOME=/Library/Java/JavaVirtualMachines/temurin-17.jdk/Contents/Home
./gradlew build
```

If you'd like, update this README with additional environment setup notes or CI instructions.
