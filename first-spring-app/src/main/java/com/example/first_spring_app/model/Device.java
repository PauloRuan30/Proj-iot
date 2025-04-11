package com.example.first_spring_app.model;

import jakarta.persistence.*;
import lombok.Data;

@Entity
@Data
public class Device {

    @Id
    @GeneratedValue(strategy = GenerationType.IDENTITY)
    private long id;

    private String name;
    private String type;
    private String location;
}