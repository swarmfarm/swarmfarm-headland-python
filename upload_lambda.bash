#!/bin/bash

file_path="headland_deploy.zip"
lambda_name="headland_creation_v2"

aws s3 cp  "$file_path" "s3://headland-creation-results/$file_path"
aws lambda update-function-code --function-name headland_creation_v2 --s3-bucket headland-creation-results --s3-key "$file_path"

git_tag=$(git describe --tags --exact-match 2>/dev/null)

# if we are on a tagged release, make a version and set prod alias
if [ -n "$git_tag" ]; then
    read -r -p "Make a version and create a release alias? (y/n): " yn
    if [ "$yn" != 'y' ]; then
        exit
    fi

    echo "Waiting 10 seconds to ensure lambda has finished updating"
    sleep 10 

    if ! output="$(aws lambda publish-version \
        --function-name "arn:aws:lambda:ap-southeast-2:940120858061:function:$lambda_name" \
        --description "Release $git_tag")" ; then
        >&2 echo "Did not create a new lambda version"
        exit 1
    fi

    version="$(echo "$output" | jq '.Version | tonumber')"
    if [ -z "$version" ]; then
        >&2 echo "Error parsing version setting output - it was $output"
        exit 1
    fi

    lambda_tagged_name="$(echo "$git_tag" | sed -e 's/\./_/g')"
    echo "Tagging and bagging for ${lambda_tagged_name}"

    aws lambda create-alias  --function-name "$lambda_name" --name "$lambda_tagged_name" --function-version "$version" --description "Tagged release for "https://github.com/swarmfarm/swarmfarm-headland-python/releases/tag/${git_tag}""

    read -r -p "Update the prod alias? (y/n): " yn

    if [ "$yn" == 'y' ]; then
        aws lambda update-alias \
            --function-name "arn:aws:lambda:ap-southeast-2:940120858061:function:$lambda_name" \
            --name 'prod' \
            --function-version "$version"
    fi
else
    echo "Not a tagged release, uploaded to \$LATEST without version"
fi