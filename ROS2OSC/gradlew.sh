ln -s `rospack find rosjava_core`/gradlew .
./gradlew clean ;
./gradlew installApp ;
./gradlew eclipse ;