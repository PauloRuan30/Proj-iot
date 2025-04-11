
FROM openjdk:21-slim
RUN mkdir /app
WORKDIR /app
COPY target/*jar /app/app.jar
ENTRYPOINT ["java", "-jar", "app.jar"]