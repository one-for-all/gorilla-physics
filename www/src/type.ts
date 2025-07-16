export type FloatArrayType = Float32Array | Float64Array;

export type FloatArrayConstructor = typeof Float32Array | typeof Float64Array;

export let FloatArray: FloatArrayConstructor = Float64Array; // or Float32Array
