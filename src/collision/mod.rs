pub mod cuboid;
pub mod epa;
pub mod gjk;
pub mod polytope;

pub fn addEdgeIfNotExisting(E: &mut Vec<(usize, usize)>, edge: (usize, usize)) {
    // remove reverse edge
    let index = E.iter().position(|x| *x == (edge.1, edge.0));
    if let Some(index) = index {
        E.remove(index);
    } else {
        E.push(edge);
    }

    // procedure to perform if E is HashSet
    // if !E.remove(&(edge.1, edge.0)) {
    //     // add if not existing
    //     E.insert(edge);
    // }
}
