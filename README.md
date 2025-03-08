# Trajectory Tracking

## Getting started

### Building the Jar

This project uses the [Shadow](https://github.com/johnrengelman/shadow) plugin for combining dependency classes and
resources into a single output Jar. To build the Jar, run the following command:

```bash
./gradlew shadowJar
```

The output Jar is located at `build/libs/trajectory-tracking-1.0-SNAPSHOT-all.jar`

## Trajectory Datasets

### GeoLife

This GPS trajectory dataset was collected in (Microsoft Research Asia) Geolife project by 182 users in a period of over three years (from April 2007 to August 2012).

https://www.microsoft.com/en-us/download/details.aspx?id=52367

### Mopsi

Mopsi contains 6,779 routes (7,850,387 points) recorded by 51 users in a period from 2008 to 2014.
Most routes are in Finland, in Joensuu region.

http://cs.uef.fi/mopsi/routes/dataset/

## Dependencies

- Java 17
- Gradle 7.4
- GeographicLib-Java 2.0
- junit 5.9.1

