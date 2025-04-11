package com.example.first_spring_app.service;

import com.example.first_spring_app.model.Device;
import com.example.first_spring_app.repository.DeviceRepository;
import org.springframework.stereotype.Service;

import java.util.List;

@Service
public class DeviceService {
    private final DeviceRepository repository;

    public DeviceService(DeviceRepository repository) {
        this.repository = repository;
    }

    public List<Device> listAll() {
        return repository.findAll();
    }

    public Device get(Long id) {
        return repository.findById(id).orElse(null);
    }

    public Device save(Device device) {
        return repository.save(device);
    }

    public void delete(Long id) {
        repository.deleteById(id);
    }
}
