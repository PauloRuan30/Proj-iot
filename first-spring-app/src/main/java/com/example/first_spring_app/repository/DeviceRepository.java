package com.example.first_spring_app.repository;

import com.example.first_spring_app.model.Device;
import org.springframework.data.jpa.repository.JpaRepository;

public interface DeviceRepository extends JpaRepository<Device, Long> {
}
