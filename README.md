# Trajectory Tracking

## Getting started

### Building the Jar

This project uses the [Shadow](https://github.com/johnrengelman/shadow) plugin for combining dependency classes and
resources into a single output Jar. To build the Jar, run the following command:

```bash
./gradlew shadowJar
```

The output Jar is located at `build/libs/trajectory-tracking-1.0-SNAPSHOT-all.jar`

## Dependencies

- Java 17
- Gradle 7.4
- GeographicLib-Java 2.0
- junit 5.9.1

