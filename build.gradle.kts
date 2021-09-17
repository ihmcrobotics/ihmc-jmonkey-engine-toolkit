plugins {
   id("us.ihmc.ihmc-build")
   id("us.ihmc.ihmc-ci") version "7.4"
   id("us.ihmc.ihmc-cd") version "1.21"
   id("us.ihmc.log-tools-plugin") version "0.6.1"
}

ihmc {
   group = "us.ihmc"
   version = "0.19.7"
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
   api("us.ihmc:jme3-core:3.2.0-171209")
   api("us.ihmc:jme3-desktop:3.2.0-171209")
   api("us.ihmc:jme3-terrain:3.2.0-171209")
   api("us.ihmc:jme3-plugins:3.2.0-171209")
   api("us.ihmc:jme3-dae:3.2.0-171209")
   // Only one version of lwjgl can be used at a time (sealed JARs), we require 2.9.3
   // for Canvas
   // api("us.ihmc:jme3-lwjgl3:3.2.0-171209")
   api("us.ihmc:jme3-lwjgl:3.2.0-171209") {
      //Exclude incompatible version of jinput
      exclude("net.java.jinput")
   }
   api("com.vividsolutions:jts:1.13")
   api("com.google.guava:guava:18.0")
   api("org.apache.commons:commons-lang3:3.11")

   api("us.ihmc:euclid:0.17.0")
   api("us.ihmc:euclid-shape:0.17.0")
   api("us.ihmc:ihmc-commons:0.30.4")
   api("us.ihmc:ihmc-graphics-description:0.19.4")
}

testDependencies {
   api("us.ihmc:ihmc-commons-testing:0.30.4")
}
