#!/bin/bash

curl -X POST \
-H "Content-Type: application/json" \
-u "$1":"$2" \
-d @$3 "https://gateway.watsonplatform.net/natural-language-understanding/api/v1/analyze?version=2018-03-16"
