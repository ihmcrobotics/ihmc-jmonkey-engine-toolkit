plugins {
   id("us.ihmc.ihmc-build")
   id("us.ihmc.ihmc-ci") version "7.6"
   id("us.ihmc.ihmc-cd") version "1.23"
   id("us.ihmc.log-tools-plugin") version "0.6.3"
}

ihmc {
   group = "us.ihmc"
   version = "0.19.8"
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
val jmeVersion = "3.5.0-stable"

mainDependencies {
   api("org.jmonkeyengine:jme3-core:$jmeVersion")
   api("org.jmonkeyengine:jme3-desktop:$jmeVersion")
   api("org.jmonkeyengine:jme3-terrain:$jmeVersion")
   api("org.jmonkeyengine:jme3-plugins:$jmeVersion")
   api("us.ihmc:jme3-dae:$jmeVersion")
   // Only one version of lwjgl can be used at a time (sealed JARs), we require 2.9.3
   // for Canvas
   // api("org.jmonkeyengine:jme3-lwjgl3:$jmeVersion")
   api("org.jmonkeyengine:jme3-lwjgl:$jmeVersion") {
      //Exclude incompatible version of jinput
      exclude("net.java.jinput")
   }
   api("com.vividsolutions:jts:1.13")
   api("com.google.guava:guava:18.0")

   api("us.ihmc:euclid:0.17.2")
   api("us.ihmc:euclid-shape:0.17.2")
   api("us.ihmc:ihmc-commons:0.30.6")
   api("us.ihmc:ihmc-graphics-description:0.19.4")
}

testDependencies {
   api("us.ihmc:ihmc-commons-testing:0.30.6")
}
