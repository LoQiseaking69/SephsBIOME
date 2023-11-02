
from tensorflow import keras
from tensorflow.keras import layers, regularizers, optimizers

def create_neural_network_model():
    # Define two separate input layers; shape based on actual sensor data dimensions
    input_dense = keras.Input(shape=(10,), name='Crown_Dense')  # Example reduced shape
    input_sparse = keras.Input(shape=(10,), name='Crown_Sparse')  # Example reduced shape

    # Dense Pathway; optimized for real-time and computational efficiency
    x_dense = layers.Dense(32, activation='relu', kernel_regularizer=regularizers.l1_l2(l1=0.001, l2=0.001))(input_dense)
    x_wisdom_dense = layers.Dense(32)(x_dense)
    x_wisdom_dense = layers.BatchNormalization()(x_wisdom_dense)
    x_wisdom_dense = layers.LeakyReLU()(x_wisdom_dense)
    x_foundation_dense = layers.Dense(64, activation='relu')(x_wisdom_dense)

    # Sparse Pathway; optimized for real-time and computational efficiency
    x_sparse = layers.Dense(32, kernel_regularizer=regularizers.l1_l2(l1=0.001, l2=0.001))(input_sparse)
    x_wisdom_sparse = layers.Dense(32)(x_sparse)
    x_wisdom_sparse = layers.BatchNormalization()(x_wisdom_sparse)
    x_wisdom_sparse = layers.LeakyReLU()(x_wisdom_sparse)
    x_foundation_sparse = layers.Dense(64)(x_wisdom_sparse)

    # Data Fusion; optimized for real-time operation
    x_fused = layers.Concatenate()([x_foundation_dense, x_foundation_sparse])
    x_fused = layers.Dropout(0.2)(layers.Dense(128, activation='relu')(x_fused))

    # Output and Reward Layers; optimized for computational efficiency and real-time operation
    output_layer = layers.Dense(5, activation='softmax')(x_fused)  # Example: 5 classes
    reward_layer = layers.Dense(1)(x_fused)  # Kept the same, assuming it's essential

    # Create the model; additional metrics or compile options can be added
    model = keras.Model(inputs=[input_dense, input_sparse], outputs=[output_layer, reward_layer])

    # Advanced Optimizer
    opt = optimizers.Adam(learning_rate=0.001)

    # Compile the model
    model.compile(optimizer=opt, loss='categorical_crossentropy', metrics=['accuracy'])

    return model
