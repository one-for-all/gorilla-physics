@group(0) @binding(0)
var<storage, read> dq: array<f32>;

@group(0) @binding(1)
var<storage, read> tetrahedra: array<vec4<u32>>;

@group(0) @binding(2)
var<storage, read> Bs: array<mat3x3f>;

@group(0) @binding(3)
var<storage, read> F_invs: array<mat3x3f>;

@group(0) @binding(4)
var<storage, read> Js: array<f32>;

@group(0) @binding(5)
var<storage, read> W_over_6s: array<f32>;

@group(0) @binding(6)
var<storage, read_write> output_dH: array<mat3x3f>;

@group(0) @binding(7)
var<storage, read> params: vec2f; // mu and lambda


@compute @workgroup_size(64)
fn compute_dH(@builtin(global_invocation_id) global_id: vec3<u32>) {
    if (global_id.x >= arrayLength(&output_dH)) {
        return;
    }

    let index = global_id.x;
    let i_vi = tetrahedra[index][0] * 3;
    let i_vj = tetrahedra[index][1] * 3;
    let i_vk = tetrahedra[index][2] * 3;
    let i_vl = tetrahedra[index][3] * 3;

    let qlx = dq[i_vl];
    let qly = dq[i_vl + 1];
    let qlz = dq[i_vl + 2];
    let dD = mat3x3<f32>(
        vec3<f32>(qlx - dq[i_vi], qly - dq[i_vi + 1], qlz - dq[i_vi + 2]),
        vec3<f32>(qlx - dq[i_vj], qly - dq[i_vj + 1], qlz - dq[i_vj + 2]),
        vec3<f32>(qlx - dq[i_vk], qly - dq[i_vk + 1], qlz - dq[i_vk + 2]),
    );

    let F_inv = F_invs[index];
    let F_inv_T = transpose(F_inv);
    let B = Bs[index];
    let dF = dD * B;
    let F_inv_dF = F_inv * dF;

    let mu = params[0];
    let lambda = params[1];
    let F_inv_dF_trace = F_inv_dF[0][0] + F_inv_dF[1][1] + F_inv_dF[2][2];
    let dP = mu * dF 
        + (mu - lambda * log(Js[index])) * F_inv_T * transpose(F_inv_dF) 
        + lambda * F_inv_dF_trace * F_inv_T;
    let dH = W_over_6s[index] * dP * transpose(B);

    output_dH[index] = dH;
}