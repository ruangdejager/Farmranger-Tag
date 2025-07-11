These are the .h and .c source files from the nanopb-0.3.9.8 release, but with necessary edits made to read the field definitions from program memory (rather than RAM as per nanopb implementation up to this point).

pb.h contains the macro definitions, and the .c files contains the implementations of these.

Typically for this definition:
	const pb_field_t *field
This usage:
	(PB_ATYPE(field->type) == PB_ATYPE_STATIC && allocsize > field->data_size))
Will be replaced by this:
	(PB_ATYPE(STRUCT_VAL_byte(field, type)) == PB_ATYPE_STATIC && allocsize > STRUCT_VAL_word(field, data_size)))