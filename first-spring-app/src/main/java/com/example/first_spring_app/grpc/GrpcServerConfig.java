package com.example.first_spring_app.grpc;

import io.grpc.Server;
import io.grpc.ServerBuilder;
import org.springframework.context.annotation.Bean;
import org.springframework.context.annotation.Configuration;

@Configuration
public class GrpcServerConfig {

    private final DeviceServiceGrpcImpl deviceServiceGrpcImpl;

    public GrpcServerConfig(DeviceServiceGrpcImpl deviceServiceGrpcImpl) {
        this.deviceServiceGrpcImpl = deviceServiceGrpcImpl;
    }

    @Bean
    public Server grpcServer() {
        return ServerBuilder.forPort(9090) // porta gRPC
                .addService(deviceServiceGrpcImpl)
                .build();
    }

    @Bean
public void startGrpcServer(Server grpcServer) {
    new Thread(() -> {
        try {
            grpcServer.start();
            System.out.println("gRPC server started on port " + grpcServer.getPort());
            grpcServer.awaitTermination();
        } catch (Exception e) {
            e.printStackTrace();
        }
    }).start();
}

}
