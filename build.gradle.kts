plugins {
    java
    id("com.github.johnrengelman.shadow") version "7.1.2"
}

group = "org.act"
version = "1.0-SNAPSHOT"

java {
    sourceCompatibility = JavaVersion.VERSION_17
    targetCompatibility = JavaVersion.VERSION_17
}

tasks.compileJava {
    options.compilerArgs.add("-Xlint:unchecked")
}

repositories {
    // maven(url = "https://maven.aliyun.com/repository/central/")
    mavenCentral()
}

val jgraphtVersion = "1.5.1"

dependencies {
    implementation("net.sf.geographiclib:GeographicLib-Java:2.0")
    testImplementation(platform("org.junit:junit-bom:5.9.1"))
    testImplementation("org.junit.jupiter:junit-jupiter")
}

tasks.test {
    useJUnitPlatform()
}