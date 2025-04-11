package com.example.first_spring_app.controller;

import com.example.first_spring_app.model.Device;
import com.example.first_spring_app.service.DeviceService;
import org.springframework.web.bind.annotation.*;

import java.util.List;

@RestController
@RequestMapping("/devices")
public class DeviceController {

    private final DeviceService service;

    public DeviceController(DeviceService service) {
        this.service = service;
    }

    @GetMapping
    public List<Device> getAll() {
        return service.listAll();
    }

    @GetMapping("/{id}")
    public Device getOne(@PathVariable Long id) {
        return service.get(id);
    }

    @PostMapping
    public Device create(@RequestBody Device device) {
        return service.save(device);
    }

    @PutMapping("/{id}")
    public Device update(@PathVariable Long id, @RequestBody Device device) {
        device.setId(id);
        return service.save(device);
    }

    @DeleteMapping("/{id}")
    public void delete(@PathVariable Long id) {
        service.delete(id);
    }
}