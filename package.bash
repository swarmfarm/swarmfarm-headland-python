#!/bin/bash
set -e

file_path="headland_deploy.zip"

rm -f $file_path
rm -rf ./package/*
pip install -r requirements.txt -t ./package

cd package

zip -r ../$file_path .

cd ..
zip $file_path lambda_function.py
zip -r $file_path app/