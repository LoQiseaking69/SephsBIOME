pub mod genome {
    use rand::prelude::*;

    #[derive(Debug, Clone)]
    pub enum GeneType {
        Neuron,
        Sensor,
        Action,
    }

    #[derive(Debug, Clone)]
    pub struct Gene {
        source_type: GeneType,
        sink_type: GeneType,
        weight: f64,
        expression_level: f64,
    }

    impl Gene {
        pub fn new(source_type: GeneType, sink_type: GeneType, weight: f64, expression_level: f64) -> Self {
            Gene { source_type, sink_type, weight, expression_level }
        }

        pub fn mutate(&mut self) {
            let mut rng = rand::thread_rng();
            self.weight += rng.gen_range(-0.05..0.05);
            self.weight = self.weight.clamp(-1.0, 1.0);

            if rng.gen_bool(0.1) {
                self.expression_level += rng.gen_range(-0.1..0.1);
                self.expression_level = self.expression_level.clamp(0.0, 1.0);
            }
        }
    }
}
