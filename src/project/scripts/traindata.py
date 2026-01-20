import cv2
import numpy as np
import os
import sys
import tensorflow as tf
from tensorflow.keras import backend as K
from tensorflow.keras.layers import (Input, ConvLSTM2D, BatchNormalization, Dense,
                                     Flatten, Reshape, RepeatVector, TimeDistributed,
                                     Conv3D, Conv2D, Conv2DTranspose, UpSampling2D)
from tensorflow.keras.models import Model
from tensorflow.keras.optimizers import Adam
from google.colab import drive
from tensorflow.keras.callbacks import EarlyStopping, ModelCheckpoint

drive.mount('/content/drive')

def create_sequences(video_path, seq_length=30, img_size=(64, 64), stride=5, max_sequences=3000):
    if not os.path.exists(video_path):
        print(f"Error: File not found at {video_path}")
        return None

    cap = cv2.VideoCapture(video_path)
    frames = []
    sequences = []

    # Get total frame count for progress bar (optional)
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    print(f"Processing {video_path}...")
    print(f"Total Frames in video: {total_frames}")

    processed_count = 0

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # Resize
        frame = cv2.resize(frame, img_size)
        # Normalize (Doing this here is cheaper on memory than doing it on the big array later)
        frame = frame.astype('float32') / 255.0
        frames.append(frame)

        # Buffer management
        if len(frames) == seq_length:
            # OPTIMIZATION: Only save every Nth sequence (stride)
            if processed_count % stride == 0:
                sequences.append(np.array(frames))

                # SAFETY: Stop if we hit the limit to prevent crash
                if len(sequences) >= max_sequences:
                    print(f"Warning: Reached max_sequences limit ({max_sequences}). Stopping to save RAM.")
                    break

            frames.pop(0) # Slide window
            processed_count += 1

            # Simple progress indicator
            if processed_count % 100 == 0:
                sys.stdout.write(f"\rProcessed: {processed_count} frames | Sequences: {len(sequences)}")
                sys.stdout.flush()

    cap.release()
    print("\nDone.")

    if not sequences:
        print("Error: Video was too short to create a sequence.")
        return None

    return np.array(sequences)

# --- RUNNING IT ---
video_path = '/content/drive/MyDrive/projectvid/Project.mp4'

# 1. Stride=5 means we take 1 sequence every 5 frames (Reduces RAM by 5x)
# 2. max_sequences=2000 ensures we don't exceed ~3GB RAM
data = create_sequences(video_path, seq_length=30, stride=5, max_sequences=2000)

if data is not None:
    print(f"Final Data Shape: {data.shape}")

seq_length = 30
img_height = 64
img_width = 64
channels = 3
input_shape = (seq_length, img_height, img_width, channels)
latent_dim = 128
batch_size = 8  # Low batch size because video data consumes VRAM fast
epochs = 20

# --- 2. SAMPLING FUNCTION (The "Variational" part) ---
def sampling(args):
    z_mean, z_log_var = args
    batch = K.shape(z_mean)[0]
    dim = K.int_shape(z_mean)[1]
    epsilon = K.random_normal(shape=(batch, dim))
    return z_mean + K.exp(0.5 * z_log_var) * epsilon

# --- 3. BUILD THE OPTIMIZED ENCODER ---
encoder_inputs = Input(shape=input_shape)

# Step 1: ConvLSTM to handle time (keeps spatial dims)
x = ConvLSTM2D(filters=16, kernel_size=(3, 3), padding='same', return_sequences=False, activation='relu')(encoder_inputs)
x = BatchNormalization()(x)

# Step 2: Spatial Downsampling (Drastically reduces parameter count)
# Input: (64, 64, 16) -> Output: (32, 32, 32)
x = Conv2D(32, (3, 3), strides=2, padding='same', activation='relu')(x)
x = BatchNormalization()(x)

# Input: (32, 32, 32) -> Output: (16, 16, 64)
x = Conv2D(64, (3, 3), strides=2, padding='same', activation='relu')(x)
x = BatchNormalization()(x)

x = Flatten()(x)
x = Dense(256, activation='relu')(x)

# Latent Space
z_mean = Dense(latent_dim, name='z_mean')(x)
z_log_var = Dense(latent_dim, name='z_log_var')(x)
z = tf.keras.layers.Lambda(sampling, output_shape=(latent_dim,), name='z')([z_mean, z_log_var])

encoder = Model(encoder_inputs, [z_mean, z_log_var, z], name='encoder')
encoder.summary()

# --- 4. BUILD THE OPTIMIZED DECODER ---
decoder_inputs = Input(shape=(latent_dim,))

# Step 1: Repeat the small vector FIRST (Saves RAM)
x = RepeatVector(seq_length)(decoder_inputs) # Output: (30, 128)

# Step 2: Project to initial spatial size (16x16) for every frame
x = TimeDistributed(Dense(16 * 16 * 64, activation='relu'))(x)
x = TimeDistributed(Reshape((16, 16, 64)))(x)

# Step 3: Spatial Upsampling
# (30, 16, 16, 64) -> (30, 32, 32, 32)
x = TimeDistributed(Conv2DTranspose(32, (3, 3), strides=2, padding='same', activation='relu'))(x)
x = BatchNormalization()(x)

# (30, 32, 32, 32) -> (30, 64, 64, 16)
x = TimeDistributed(Conv2DTranspose(16, (3, 3), strides=2, padding='same', activation='relu'))(x)
x = BatchNormalization()(x)

# Step 4: Final smoothing with ConvLSTM
x = ConvLSTM2D(filters=16, kernel_size=(3, 3), padding='same', return_sequences=True, activation='relu')(x)

# Step 5: Final output (3 channels for RGB)
decoder_outputs = TimeDistributed(Conv2D(3, (3, 3), activation='sigmoid', padding='same'))(x)

decoder = Model(decoder_inputs, decoder_outputs, name='decoder')
decoder.summary()

# --- INSERT THIS CLASS BEFORE SECTION 5 ---

class VAELossLayer(tf.keras.layers.Layer):
    def __init__(self, **kwargs):
        super(VAELossLayer, self).__init__(**kwargs)

    def call(self, inputs):
        x_true, x_pred, z_mean, z_log_var = inputs

        # 1. Flatten the video data for loss calculation
        # Shape: (Batch, 30*64*64*3)
        x_true_flat = tf.reshape(x_true, [-1, 30 * 64 * 64 * 3])
        x_pred_flat = tf.reshape(x_pred, [-1, 30 * 64 * 64 * 3])

        # 2. Reconstruction Loss (Binary Crossentropy)
        reconstruction_loss = tf.keras.losses.binary_crossentropy(x_true_flat, x_pred_flat)
        reconstruction_loss *= (30 * 64 * 64 * 3) # Scale up by number of pixels

        # 3. KL Divergence Loss
        kl_loss = 1 + z_log_var - tf.square(z_mean) - tf.exp(z_log_var)
        kl_loss = tf.reduce_sum(kl_loss, axis=-1)
        kl_loss *= -0.5

        # 4. Total Loss
        total_loss = tf.reduce_mean(reconstruction_loss + kl_loss)

        # 5. Add loss to the model
        self.add_loss(total_loss)

        return x_pred

# --- 5. COMPILE VAE (UPDATED) ---

# Get the parts from the encoder
z_mean, z_log_var, z = encoder(encoder_inputs)

# Get the raw output from the decoder
raw_outputs = decoder(z)

# Pass everything through our new Loss Layer
# This calculates loss internally and returns the output
vae_outputs = VAELossLayer()([encoder_inputs, raw_outputs, z_mean, z_log_var])

# Create the final model
vae = Model(encoder_inputs, vae_outputs, name='vae')

callback = EarlyStopping(
    monitor='val_loss',
    patience=3,
    restore_best_weights=True
)

optimizer = Adam(learning_rate=0.0001, clipnorm=1.0)
# Compile (Note: loss is None because the layer handles it!)
vae.compile(optimizer=optimizer)
vae.summary()

# --- 6. TRAIN ---
print("Starting training...")
# We use x=data, y=None because the loss is calculated inside the model using the input
history = vae.fit(data, None, epochs=epochs, batch_size=batch_size, validation_split=0.1, callbacks=[callback])

# --- 7. SAVE MODEL ---
# Save weights only (safest for custom layers)
vae.save_weights('robot_anomaly_vae.weights.h5')
print("Model weights saved.")