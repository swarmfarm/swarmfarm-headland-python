# Use the official Python 3.8 image
FROM python:3.8-slim

# Install gcc, build-essential, and zip
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    gcc \
    build-essential \
    zip \
    python3-tk \
    x11-apps \
    libx11-dev \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Set the working directory
WORKDIR /app

# Copy the requirements file and install the dependencies
COPY requirements.txt ./
RUN pip install --no-cache-dir -r requirements.txt

# Specify the entrypoint for the container
ENTRYPOINT ["/bin/bash"]