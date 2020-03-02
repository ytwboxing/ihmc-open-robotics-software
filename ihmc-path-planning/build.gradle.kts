import java.io.FileWriter

plugins {
   id("us.ihmc.ihmc-build") version "0.20.1"
   id("us.ihmc.ihmc-ci") version "5.3"
   id("us.ihmc.ihmc-cd") version "1.14"
   id("us.ihmc.log-tools") version "0.3.1"
}

ihmc {
   loadProductProperties("../product.properties")
   maintainer = "Daniel Duran (dduran@ihmc.us)"

   configureDependencyResolution()
   configurePublications()
}

mainDependencies {
   api("org.jgrapht:jgrapht-core:0.9.0")

   api("us.ihmc:ihmc-commons:0.26.6")
   api("us.ihmc:euclid-frame:0.12.2")
   api("us.ihmc:euclid-shape:0.12.2")
   api("us.ihmc:ihmc-javafx-toolkit:0.14.1") {
      exclude(group = "org.slf4j", module = "slf4j-simple")
   }
   api("us.ihmc:ihmc-yovariables:0.4.0")
   api("us.ihmc:ihmc-convex-optimization:0.13.0")
   api("us.ihmc:robot-environment-awareness:source")
}

visualizersDependencies {
   api(ihmc.sourceSetProject("main"))
   api(ihmc.sourceSetProject("data-sets"))

   api("us.ihmc:ihmc-javafx-toolkit:0.14.1") {
      exclude(group = "org.slf4j", module = "slf4j-simple")
   }
   api("us.ihmc:robot-environment-awareness-application:source")
   api("us.ihmc:ihmc-communication:source")
   api("us.ihmc:ihmc-robot-models-visualizers:source")
}

testDependencies {
   api(ihmc.sourceSetProject("visualizers"))
   api(ihmc.sourceSetProject("data-sets"))

   api("us.ihmc:ihmc-commons-testing:0.26.6")
   api("us.ihmc:simulation-construction-set:0.14.0")
   api("us.ihmc:robot-environment-awareness-application:source")
   api("us.ihmc:ihmc-robotics-toolkit-test:source")
}

//ihmc.sourceSetProject("data-sets").dependencies {
//
//}
dataSetsDependencies {
   api(ihmc.sourceSetProject("main"))

   api("us.ihmc:ihmc-commons-testing:0.26.6")
   api("us.ihmc:simulation-construction-set-tools:source")
   api("us.ihmc:robot-environment-awareness:source")
}

tasks.create("updateDataSetList") {
   doFirst {
      var resourcesDir = "src/data-sets/resources/us/ihmc/pathPlanning/dataSets"
      var srcDir = "src/data-sets/java/us/ihmc/pathPlanning"
      var className = "DataSetName"

      var dataSetDir = File(resourcesDir)
      var dataSetList = files(dataSetDir.listFiles().sort()) // how to do reverse here?

      var classFile = File(srcDir + "/" + className + ".java")
      println(classFile.getAbsolutePath())

      if(classFile.exists())
         classFile.delete()
      classFile.createNewFile()

      var fileWriter = FileWriter(classFile, true)
      fileWriter.write("package us.ihmc.pathPlanning;\n\n")
      fileWriter.write("public enum " + className + "\n")
      fileWriter.write("{\n")

      dataSetList.forEach {
         fileWriter.write("\t_" + it.name + ",\n")
      }

      fileWriter.write("}")
      fileWriter.flush()
      fileWriter.close()
   }
}