plugins {
   id("us.ihmc.ihmc-build") version "0.22.0"
   id("us.ihmc.ihmc-ci") version "6.8"
   id("us.ihmc.ihmc-cd") version "1.14"
   id("us.ihmc.log-tools-plugin") version "0.5.0"
}

ihmc {
   group = "us.ihmc"
   version = "0.19.1"
   vcsUrl = "https://github.com/ihmcrobotics/ihmc-jmonkey-engine-toolkit"
   openSource = true

   configureDependencyResolution()
   configurePublications()
}

categories.configure("fast").enableAssertions = false
val jme = categories.configure("jme")
jme.enableAssertions = false
jme.minHeapSizeGB = 2
jme.maxHeapSizeGB = 6
jme.forkEvery = 1
jme.maxParallelForks = 1

mainDependencies {
   var jmeVersion = "3.3.0-stable"
   api("org.jmonkeyengine:jme3-core:$jmeVersion")
   api("org.jmonkeyengine:jme3-desktop:$jmeVersion")
   api("org.jmonkeyengine:jme3-terrain:$jmeVersion")
   api("org.jmonkeyengine:jme3-plugins:$jmeVersion")
   // Only one version of lwjgl can be used at a time (sealed JARs), we require 2.9.3 for AWT LwjglCanvas
   var lwjglVersion = "lwjgl3";
   api("org.jmonkeyengine:jme3-$lwjglVersion:$jmeVersion") {
      exclude(group = "net.java.jinput", module = "jinput") // Exclude incompatible version of jinput
   }
   api("com.vividsolutions:jts:1.13")
   api("com.google.guava:guava:18.0")
   api("org.apache.commons:commons-lang3:3.9")

   api("us.ihmc:euclid:0.15.1")
   api("us.ihmc:euclid-shape:0.15.1")
   api("us.ihmc:ihmc-commons:0.30.2")
   api("us.ihmc:ihmc-graphics-description:0.19.1")
}

testDependencies {
   api("us.ihmc:ihmc-commons-testing:0.30.2")
}
