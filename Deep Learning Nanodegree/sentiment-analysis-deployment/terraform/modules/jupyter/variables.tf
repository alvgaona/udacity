variable "name" {
  type = "string"
  description = "AWS SageMaker Notebook instance name"
}

variable "role_arn" {
  type = "string"
  description = "IAM role to apply upon notebook instance"
}

variable "instance_type" {
  type = "string"
  default = "ml.t2.medium"
  description = "AWS SageMaker Notebook instance type"
}