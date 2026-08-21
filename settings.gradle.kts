pluginManagement {
    repositories {
        google()
        mavenCentral()
        gradlePluginPortal()
    }
}

dependencyResolutionManagement {
    repositoriesMode.set(RepositoriesMode.FAIL_ON_PROJECT_REPOS)
    repositories {
        google()
        mavenCentral()
        maven("https://jitpack.io")  // 🌟 就是把这行加在这里！
    }
}

rootProject.name = "Cartographer-Android"
include(":app")
include(":cartographer-sdk")
