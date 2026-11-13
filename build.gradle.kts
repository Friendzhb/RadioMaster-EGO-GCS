plugins {
    id("com.android.application") version "8.5.2" apply false
    id("org.jetbrains.kotlin.android") version "1.9.24" apply false
    id("com.diffplug.spotless") version "6.25.0"
}

spotless {
    kotlin {
        target("app/src/**/*.kt")
        ktlint("1.2.1")
    }
    kotlinGradle {
        target("*.gradle.kts", "app/*.gradle.kts")
        ktlint("1.2.1")
    }
    format("misc") {
        target(".editorconfig", ".gitignore", ".gitattributes", "README.md", "docs/*.md", ".github/workflows/*.yml")
        trimTrailingWhitespace()
        endWithNewline()
    }
}
