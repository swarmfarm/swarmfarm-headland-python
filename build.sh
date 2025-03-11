#!/bin/bash
set -e

# Ensure you're in the /app directory
cd /app

# Remove the existing zip file if it exists
rm -f headland_deploy.zip
rm -rf ./package
mkdir package
# Copy dependencies to the package directory
cp -r /deps/* ./package

# Change to the package directory and zip its contents
cd package
zip -r ../headland_deploy.zip .

# Move back to the /app directory
cd ..

# Add the lambda_function.py and app/ directory to the zip file
zip headland_deploy.zip lambda_function.py
zip -r headland_deploy.zip app/