package com.example.first_spring_app.grpc;

import com.example.DeviceOuterClass.DeviceRequest;
import com.example.DeviceOuterClass.DeviceList;
import com.example.DeviceOuterClass.Device;
import com.example.DeviceOuterClass.Empty;
import com.example.DeviceServiceGrpc;
import com.example.first_spring_app.service.DeviceService;

import io.grpc.stub.StreamObserver;
import org.springframework.stereotype.Service;

@Service
public class DeviceServiceGrpcImpl extends DeviceServiceGrpc.DeviceServiceImplBase {

    private final DeviceService deviceService;

    public DeviceServiceGrpcImpl(DeviceService deviceService) {
        this.deviceService = deviceService;
    }

    private Device toGrpcDevice(com.example.first_spring_app.model.Device device) {
        return Device.newBuilder()
                .setId(device.getId())
                .setName(device.getName())
                .setType(device.getType())
                .setLocation(device.getLocation())
                .build();
    }

    private com.example.first_spring_app.model.Device toModelDevice(Device device) {
        com.example.first_spring_app.model.Device model = new com.example.first_spring_app.model.Device();
        model.setId(device.getId());
        model.setName(device.getName());
        model.setType(device.getType());
        model.setLocation(device.getLocation());
        return model;
    }

    @Override
    public void listDevices(Empty request, StreamObserver<DeviceList> responseObserver) {
        DeviceList.Builder deviceListBuilder = DeviceList.newBuilder();
        deviceService.listAll().forEach(device -> {
            deviceListBuilder.addDevices(toGrpcDevice(device));
        });
        responseObserver.onNext(deviceListBuilder.build());
        responseObserver.onCompleted();
    }

    @Override
    public void getDevice(DeviceRequest request, StreamObserver<Device> responseObserver) {
        com.example.first_spring_app.model.Device modelDevice = deviceService.get(request.getId());
        if (modelDevice != null) {
            responseObserver.onNext(toGrpcDevice(modelDevice));
        } else {
            responseObserver.onError(new RuntimeException("Device not found"));
        }
        responseObserver.onCompleted();
    }

    @Override
    public void createDevice(Device request, StreamObserver<Device> responseObserver) {
        com.example.first_spring_app.model.Device saved = deviceService.save(toModelDevice(request));
        responseObserver.onNext(toGrpcDevice(saved));
        responseObserver.onCompleted();
    }

    @Override
    public void updateDevice(Device request, StreamObserver<Device> responseObserver) {
        com.example.first_spring_app.model.Device updated = deviceService.save(toModelDevice(request));
        responseObserver.onNext(toGrpcDevice(updated));
        responseObserver.onCompleted();
    }

    @Override
    public void deleteDevice(DeviceRequest request, StreamObserver<Empty> responseObserver) {
        deviceService.delete(request.getId());
        responseObserver.onNext(Empty.newBuilder().build());
        responseObserver.onCompleted();
    }
}
