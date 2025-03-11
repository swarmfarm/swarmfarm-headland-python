#!/bin/bash
set -e

rm -f headland-deploy.zip
rm -rf ./package/*
pip install -r requirements.txt -t ./package

cd package

zip -r ../headland_deploy.zip .

cd ..
zip headland_deploy.zip lambda_function.py
zip -r headland_deploy.zip app/