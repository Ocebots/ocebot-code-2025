#!/usr/bin/env zsh

echo "#!/usr/bin/env zsh\n\n./gradlew spotlessApply" > .git/hooks/pre-commit
