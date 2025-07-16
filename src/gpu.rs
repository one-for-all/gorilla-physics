use std::{collections::HashMap, num::NonZeroU64};

use na::Matrix3;
use wgpu::{util::DeviceExt, BindGroup, Buffer, ComputePipeline, Device, Queue, ShaderModule};

use crate::types::Float;

/// Convenience container of GPU related objects
pub struct WgpuContext {
    pub device: Device,
    pub queue: Queue,
    pub module: ShaderModule,
    pub pipeline: ComputePipeline,
    pub bind_group: BindGroup,
    pub input_buffers: HashMap<&'static str, Buffer>,
    pub output_buffers: HashMap<&'static str, Buffer>,
    pub download_buffers: HashMap<&'static str, Buffer>,
}

/// Initialize the interface to the GPU
pub async fn async_initialize_gpu() -> (Device, Queue) {
    let instance = wgpu::Instance::new(&wgpu::InstanceDescriptor::default());

    let adapter = instance
        .request_adapter(&wgpu::RequestAdapterOptions::default())
        .await
        .expect("Failed to create WebGPU adapter");

    let downlevel_capabilities = adapter.get_downlevel_capabilities();
    if !downlevel_capabilities
        .flags
        .contains(wgpu::DownlevelFlags::COMPUTE_SHADERS)
    {
        panic!("Adapter does not support compute shaders");
    }

    let mut limits = wgpu::Limits::downlevel_defaults();
    limits.max_storage_buffers_per_shader_stage = 8;
    let (device, queue) = adapter
        .request_device(&wgpu::DeviceDescriptor {
            label: None,
            required_features: wgpu::Features::empty(),
            required_limits: limits,
            memory_hints: wgpu::MemoryHints::MemoryUsage,
            trace: wgpu::Trace::Off,
        })
        .await
        .expect("Failed to create device");

    (device, queue)
}

/// Struct to passing 3x3 Matrix between Rust and wgsl
#[repr(C)]
#[derive(Copy, Clone, bytemuck::Pod, bytemuck::Zeroable, Debug)]
pub struct Matrix3x3 {
    pub cols: [[Float; 4]; 3], // Column-major for WGSL, each column has 4 values to account for wgsl padding
} // TODO/Note: the type Float needs to correspond to wgsl type

/// Set up the pipeline for computing dH
pub fn setup_compute_dH_pipeline(
    device: &Device,
    n_vertices: usize,
    tetrahedra: &Vec<Vec<usize>>,
    padded_Bs: &Vec<Matrix3x3>,
    Ws: &Vec<Float>,
    mu: Float,
    lambda: Float,
) -> (
    ShaderModule,
    ComputePipeline,
    BindGroup,
    Buffer,
    Buffer,
    Buffer,
    Buffer,
    Buffer,
) {
    let shader_module = device.create_shader_module(wgpu::include_wgsl!("fem.wgsl"));

    let input_dq_buffer = device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("input dq buffer"),
        size: (n_vertices * 3 * std::mem::size_of::<f32>()) as wgpu::BufferAddress,
        usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_DST,
        mapped_at_creation: false,
    });

    let tetrahedra_flatten: Vec<u32> = tetrahedra
        .iter()
        .flat_map(|t| [t[0] as u32, t[1] as u32, t[2] as u32, t[3] as u32])
        .collect();
    let input_tetrahedra_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
        label: None,
        contents: bytemuck::cast_slice(&tetrahedra_flatten),
        usage: wgpu::BufferUsages::STORAGE,
    });

    let input_Bs_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
        label: Some("input Bs buffer"),
        contents: bytemuck::cast_slice(&padded_Bs),
        usage: wgpu::BufferUsages::STORAGE,
    });

    let input_F_invs_buffer = device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("input F_invs buffer"),
        size: (tetrahedra.len() * std::mem::size_of::<Matrix3x3>()) as wgpu::BufferAddress,
        usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_DST,
        mapped_at_creation: false,
    });

    let input_Js_buffer = device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("input Js buffer"),
        size: (tetrahedra.len() * std::mem::size_of::<f32>()) as wgpu::BufferAddress,
        usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_DST,
        mapped_at_creation: false,
    });

    let input_Ws_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
        label: Some("input Ws buffer"),
        contents: bytemuck::cast_slice(&Ws.as_slice()),
        usage: wgpu::BufferUsages::STORAGE,
    });

    let output_dH_buffer = device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("output dF buffer"),
        size: (tetrahedra.len() * std::mem::size_of::<Matrix3x3>()) as wgpu::BufferAddress,
        usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_SRC,
        mapped_at_creation: false,
    });

    let download_dH_buffer = device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("download dH buffer"),
        size: output_dH_buffer.size(),
        usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
        mapped_at_creation: false,
    });

    let params_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
        label: Some("params buffer"),
        contents: bytemuck::cast_slice(&[mu, lambda]),
        usage: wgpu::BufferUsages::STORAGE,
    });

    let bind_group_layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
        label: None,
        entries: &[
            wgpu::BindGroupLayoutEntry {
                binding: 0,
                visibility: wgpu::ShaderStages::COMPUTE,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Storage { read_only: true },
                    min_binding_size: Some(NonZeroU64::new(4).unwrap()),
                    has_dynamic_offset: false,
                },
                count: None,
            },
            wgpu::BindGroupLayoutEntry {
                binding: 1,
                visibility: wgpu::ShaderStages::COMPUTE,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Storage { read_only: true },
                    min_binding_size: Some(
                        NonZeroU64::new(4 * std::mem::size_of::<u32>() as wgpu::BufferAddress)
                            .unwrap(),
                    ),
                    has_dynamic_offset: false,
                },
                count: None,
            },
            wgpu::BindGroupLayoutEntry {
                binding: 2,
                visibility: wgpu::ShaderStages::COMPUTE,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Storage { read_only: true },
                    min_binding_size: Some(
                        NonZeroU64::new(std::mem::size_of::<Matrix3x3>() as wgpu::BufferAddress)
                            .unwrap(),
                    ),
                    has_dynamic_offset: false,
                },
                count: None,
            },
            wgpu::BindGroupLayoutEntry {
                binding: 3,
                visibility: wgpu::ShaderStages::COMPUTE,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Storage { read_only: true },
                    min_binding_size: Some(
                        NonZeroU64::new(std::mem::size_of::<Matrix3x3>() as wgpu::BufferAddress)
                            .unwrap(),
                    ),
                    has_dynamic_offset: false,
                },
                count: None,
            },
            wgpu::BindGroupLayoutEntry {
                binding: 4,
                visibility: wgpu::ShaderStages::COMPUTE,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Storage { read_only: true },
                    min_binding_size: Some(NonZeroU64::new(4).unwrap()),
                    has_dynamic_offset: false,
                },
                count: None,
            },
            wgpu::BindGroupLayoutEntry {
                binding: 5,
                visibility: wgpu::ShaderStages::COMPUTE,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Storage { read_only: true },
                    min_binding_size: Some(NonZeroU64::new(4).unwrap()),
                    has_dynamic_offset: false,
                },
                count: None,
            },
            wgpu::BindGroupLayoutEntry {
                binding: 6,
                visibility: wgpu::ShaderStages::COMPUTE,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Storage { read_only: false },
                    min_binding_size: Some(
                        NonZeroU64::new(std::mem::size_of::<Matrix3x3>() as wgpu::BufferAddress)
                            .unwrap(),
                    ),
                    has_dynamic_offset: false,
                },
                count: None,
            },
            wgpu::BindGroupLayoutEntry {
                binding: 7,
                visibility: wgpu::ShaderStages::COMPUTE,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Storage { read_only: true },
                    min_binding_size: Some(
                        NonZeroU64::new(2 * std::mem::size_of::<f32>() as wgpu::BufferAddress)
                            .unwrap(),
                    ),
                    has_dynamic_offset: false,
                },
                count: None,
            },
        ],
    });

    let bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
        label: Some("bind group"),
        layout: &bind_group_layout,
        entries: &[
            wgpu::BindGroupEntry {
                binding: 0,
                resource: input_dq_buffer.as_entire_binding(),
            },
            wgpu::BindGroupEntry {
                binding: 1,
                resource: input_tetrahedra_buffer.as_entire_binding(),
            },
            wgpu::BindGroupEntry {
                binding: 2,
                resource: input_Bs_buffer.as_entire_binding(),
            },
            wgpu::BindGroupEntry {
                binding: 3,
                resource: input_F_invs_buffer.as_entire_binding(),
            },
            wgpu::BindGroupEntry {
                binding: 4,
                resource: input_Js_buffer.as_entire_binding(),
            },
            wgpu::BindGroupEntry {
                binding: 5,
                resource: input_Ws_buffer.as_entire_binding(),
            },
            wgpu::BindGroupEntry {
                binding: 6,
                resource: output_dH_buffer.as_entire_binding(),
            },
            wgpu::BindGroupEntry {
                binding: 7,
                resource: params_buffer.as_entire_binding(),
            },
        ],
    });

    let pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
        label: Some("pipeline layout"),
        bind_group_layouts: &[&bind_group_layout],
        push_constant_ranges: &[],
    });

    let pipeline = device.create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
        label: Some("pipeline"),
        layout: Some(&pipeline_layout),
        module: &shader_module,
        entry_point: Some("compute_dH"),
        compilation_options: wgpu::PipelineCompilationOptions::default(),
        cache: None,
    });

    (
        shader_module,
        pipeline,
        bind_group,
        input_dq_buffer,
        input_F_invs_buffer,
        input_Js_buffer,
        output_dH_buffer,
        download_dH_buffer,
    )
}

pub async fn compute_dH(
    wgpu_context: &WgpuContext,
    tetrahedra: &Vec<Vec<usize>>,
) -> Vec<Matrix3<Float>> {
    let mut encoder = wgpu_context
        .device
        .create_command_encoder(&wgpu::CommandEncoderDescriptor { label: None });
    let mut compute_pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor {
        label: None,
        timestamp_writes: None,
    });
    compute_pass.set_pipeline(&wgpu_context.pipeline);
    compute_pass.set_bind_group(0, &wgpu_context.bind_group, &[]);

    let workgroup_count = tetrahedra.len().div_ceil(64);
    compute_pass.dispatch_workgroups(workgroup_count as u32, 1, 1);

    drop(compute_pass);

    let output_dH_buffer = &wgpu_context.output_buffers["dH"];
    let download_dH_buffer = &wgpu_context.download_buffers["dH"];
    encoder.copy_buffer_to_buffer(
        &output_dH_buffer,
        0,
        &download_dH_buffer,
        0,
        output_dH_buffer.size(),
    );

    let command_buffer = encoder.finish();
    wgpu_context.queue.submit([command_buffer]);
    let buffer_dF_slice = download_dH_buffer.slice(..);

    let (sender, receiver) = flume::bounded(1);

    buffer_dF_slice.map_async(wgpu::MapMode::Read, move |r| sender.send(r).unwrap());

    wgpu_context.device.poll(wgpu::PollType::Wait).unwrap();
    receiver.recv_async().await.unwrap().unwrap();
    let data_dF = buffer_dF_slice.get_mapped_range();
    let dF: &[Matrix3x3] = bytemuck::cast_slice(&data_dF);

    dF.iter()
        .map(|m| {
            #[rustfmt::skip]
            let mat = Matrix3::new(
                m.cols[0][0], m.cols[1][0], m.cols[2][0],
                m.cols[0][1], m.cols[1][1], m.cols[2][1],
                m.cols[0][2], m.cols[1][2], m.cols[2][2],
            );
            mat
        })
        .collect()
}
