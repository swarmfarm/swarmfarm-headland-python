import json
import logging
import traceback

import boto3
from app.headlands import AlgorithmError, ErrorType, GenerationError, HeadlandCreator, make_error_list_return

logger = logging.getLogger()
logger.setLevel(logging.INFO)
s3 = boto3.resource("s3")


def lambda_handler(event, _):
    logging.info("Starting python headlands")
    try:
        executionArn = event.get("executionArn")
        body = event.get("body")

        logging.info("body:")
        logging.info(body)
        if body is None:
            raise GenerationError(ErrorType.BAD_INPUT_DATA, "Missing 'body' in payload")
        try:
            if isinstance(body, str):
                body = json.loads(body)
        except json.JSONDecodeError as e:
            # Handle JSON decoding error (e.g., log it and return an error response)
            return {
                "statusCode": 400,
                "executionArn": executionArn,
                "body": json.dumps({"message": "Invalid JSON format"}),
            }
        # Body is now just: {'path': string, 'alias': string}
        # We need to get the payload from S3
        obj = s3.Object("headland-creation-results", body.get("path"))
        objContent = obj.get()["Body"].read()
        payload = json.loads(objContent)
        settings = payload.get("settings")
        if settings is None:
            raise GenerationError(
                ErrorType.BAD_INPUT_DATA,
                "Missing 'settings' in payload",
            )
        operating_area = payload.get("operatingArea")
        if operating_area is None:
            raise GenerationError(
                ErrorType.BAD_INPUT_DATA, "Missing 'operatingArea' in payload"
            )
        obstacles = payload.get("obstacles")
        if obstacles is None:
            raise GenerationError(
                ErrorType.BAD_INPUT_DATA, "Missing 'obstacles' in payload"
            )
        tree_rows = payload.get("treeRows")
        if tree_rows is None:
            tree_rows = list()
        adjacent_tree_rows = payload.get("adjacentTreeRows")
        if adjacent_tree_rows is None:
            adjacent_tree_rows = list()
        geometryCreator = HeadlandCreator(
            settings=settings,
            operating_area=operating_area,
            obstacles=obstacles,
            tree_rows=tree_rows,
            adjacent_tree_rows=adjacent_tree_rows,
            executionArn=executionArn,
        )
        headlands, errors = geometryCreator.generate_headlands()
        logging.info("Finished python headlands")
        logging.info(headlands)
        logging.info(errors)
        metadata = payload.get("metadata")
        resultBody = json.dumps(
            {
                "result": {
                    "success": True,
                    "payload": headlands,
                    "error": errors,
                },
                "input": {"metadata": metadata},
            }
        )
        s3.Bucket("headland-creation-results").put_object(
            Key=executionArn, Body=resultBody, ContentType="application/json"
        )
        return {"success": True, "executionArn": executionArn}
    except (GenerationError, AlgorithmError) as e:
        logger.error("Error: %s", traceback.format_exc())
        # logger.log(event)
        resultBody = json.dumps(
            {
                "result": {
                    "success": False,
                    "error": make_error_list_return(e),
                },
                "input": body,
            }
        )
        s3.Bucket("headland-creation-results").put_object(
            Key=executionArn, Body=resultBody, ContentType="application/json"
        )
        return {"success": False, "executionArn": executionArn}
